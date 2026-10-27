/* NanoVDB file/raw-buffer reader. Included by tinyvdb_nanovdb.h.
 * SPDX-License-Identifier: Apache-2.0 */
static void *tvdb__nv_alloc(tvdb_nanovdb_file_t *f,size_t n) {
    return f->alloc.malloc_fn?f->alloc.malloc_fn(n,f->alloc.user_ctx):malloc(n);
}
static void tvdb__nv_free(tvdb_nanovdb_file_t *f,void *p,size_t n) {
    if (!p) return;
    if (f->alloc.free_fn) f->alloc.free_fn(p,n,f->alloc.user_ctx); else free(p);
}
static uint32_t tvdb__nv_u32(const uint8_t *p) { uint32_t v;memcpy(&v,p,4);return v; }
static uint64_t tvdb__nv_u64(const uint8_t *p) { uint64_t v;memcpy(&v,p,8);return v; }
static double tvdb__nv_f64(const uint8_t *p) { double v;memcpy(&v,p,8);return v; }

static int tvdb__nv_child_address(uint64_t base,uint64_t delta,uint64_t size,uint64_t *out) {
    if(delta<=INT64_MAX) {if(delta>size-base)return 0;*out=base+delta;}
    else {uint64_t magnitude=(~delta)+1;if(magnitude>base)return 0;*out=base-magnitude;}
    return *out>=736 && *out<=size && !(*out&7);
}
/* Bound every child pointer reachable by the PNanoVDB accessors before
 * exposing the grid. Depth is fixed by the format. The per-level visit budget
 * is the declared node count, capped by how many nodes of that size fit in the
 * grid, so repeated (aliased) child pointers cannot multiply the work beyond
 * what the input size allows. */
static int tvdb__nv_check_node(const uint8_t *p,uint64_t size,uint64_t address,int level,
    const pnanovdb_grid_type_constants_t *c,uint32_t remaining[3]) {
    uint64_t bytes=level==2?c->upper_size:level==1?c->lower_size:c->leaf_size;
    if(!remaining[level] || address>size || bytes>size-address)return 0;
    remaining[level]--;
    if(!level)return 1;
    uint32_t mask=level==2?PNANOVDB_UPPER_OFF_CHILD_MASK:PNANOVDB_LOWER_OFF_CHILD_MASK;
    uint32_t table=level==2?c->upper_off_table:c->lower_off_table,slots=level==2?32768:4096;
    for(uint32_t k=0;k<slots;k++)if(tvdb__nv_u32(p+address+mask+4*(k>>5))&(1u<<(k&31))) {
        uint64_t child,delta=tvdb__nv_u64(p+address+table+(uint64_t)k*c->table_stride);
        if(!tvdb__nv_child_address(address,delta,size,&child) ||
            !tvdb__nv_check_node(p,size,child,level-1,c,remaining))return 0;
    }
    return 1;
}

/* Append `count` zeroed grid slots, growing geometrically. The previous
 * exact-fit realloc copied the whole array on every call, making a multi-grid
 * file O(grids^2) in bytes moved; halving-vs-doubling keeps it amortized O(1).
 * Capacity is tracked in grid_count's high half via a separate field to avoid
 * changing the public struct. */
static int tvdb__nv_append(tvdb_nanovdb_file_t *f,size_t count) {
    if (count>SIZE_MAX/sizeof(*f->grids)-f->num_grids) return 0;
    size_t total=f->num_grids+count;
    size_t cap=f->grid_capacity;
    if (total>cap) {
        size_t new_cap=cap?cap*2:16;
        while (new_cap<total) {
            if (new_cap>SIZE_MAX/2) { new_cap=total; break; }
            new_cap*=2;
        }
        if (new_cap>SIZE_MAX/sizeof(*f->grids)) return 0;
        tvdb_nanovdb_grid_t *p=(tvdb_nanovdb_grid_t*)realloc(f->grids,new_cap*sizeof(*p));
        if (!p) return 0;
        /* Realloc may have moved, so the allocation-site bookkeeping used by
         * tvdb__nv_free has to be reset for the new block. */
        f->grids=p; f->grid_capacity=new_cap;
    }
    memset(f->grids+f->num_grids,0,count*sizeof(*f->grids));
    f->num_grids=total;
    return 1;
}

/* Release every grid-owned allocation. Used on the parse-failure path so a
 * rejected file does not strand the grids decoded so far. */
static void tvdb__nv_release_grids(tvdb_nanovdb_file_t *f) {
    if (!f) return;
    for (size_t i=0;i<f->num_grids;i++) {
        tvdb_nanovdb_grid_t *g=&f->grids[i];
        if (g->name) tvdb__nv_free(f,g->name,strlen(g->name)+1);
        if (g->owns_data) tvdb__nv_free(f,g->data,(size_t)g->size);
    }
    if (f->grids) free(f->grids); /* grown with realloc, not the allocator */
    f->grids=NULL; f->num_grids=0; f->grid_capacity=0;
}

static tvdb_status_t tvdb__nv_metadata(tvdb_nanovdb_file_t *f,tvdb_nanovdb_grid_t *g) {
    const uint8_t *p=g->data;
    if (g->size<736 || (tvdb__nv_u64(p)!=TVDB_NANOVDB_MAGIC_GRID && tvdb__nv_u64(p)!=TVDB_NANOVDB_MAGIC_NUMB) ||
        tvdb__nv_u64(p+32)!=g->size || (tvdb__nv_u32(p+16)>>21)!=32) return TVDB_ERROR_INVALID_DATA;
    uint32_t type=tvdb__nv_u32(p+636);
    if (!type || type>=PNANOVDB_GRID_TYPE_END) return TVDB_ERROR_UNSUPPORTED_GRID_TYPE;
    g->grid_type=type;g->grid_class=tvdb__nv_u32(p+632);g->tree_data_offset=672;
    uint64_t offsets[4];
    for (int k=0;k<4;k++) {
        uint64_t offset=tvdb__nv_u64(p+672+8*k);
        if (offset>g->size-672) return TVDB_ERROR_INVALID_DATA;
        offsets[k]=672+offset;
    }
    uint64_t root=offsets[3];
    if (root>g->size || g->size-root<32) return TVDB_ERROR_INVALID_DATA;
    const pnanovdb_grid_type_constants_t *constants=&pnanovdb_grid_type_constants[type];
    uint32_t tiles=tvdb__nv_u32(p+root+24);
    if (constants->root_size>g->size-root ||
        (uint64_t)tiles*constants->root_tile_size>g->size-root-constants->root_size) return TVDB_ERROR_INVALID_DATA;
    /* Every grid type is walked by tvdb_nanovdb_is_voxel_active, so every
     * type's child pointers must be validated. */
    {
        uint32_t remaining[3];
        for(int k=0;k<3;k++) {
            uint64_t bytes=k==2?constants->upper_size:k==1?constants->lower_size:constants->leaf_size;
            uint64_t cap=bytes?g->size/bytes:0;
            remaining[k]=tvdb__nv_u32(p+704+4*k);
            if(remaining[k]>cap)remaining[k]=(uint32_t)cap;
        }
        for(uint32_t k=0;k<tiles;k++) {
            uint64_t delta=tvdb__nv_u64(p+root+constants->root_size+(uint64_t)k*constants->root_tile_size+8),child;
            if(delta && (!tvdb__nv_child_address(root,delta,g->size,&child) ||
                !tvdb__nv_check_node(p,g->size,child,2,constants,remaining)))return TVDB_ERROR_INVALID_DATA;
        }
    }
    g->leaf_data_offset=(int64_t)offsets[0];g->lower_data_offset=(int64_t)offsets[1];
    g->upper_data_offset=(int64_t)offsets[2];g->root_data_offset=(int64_t)root;
    for (int k=0;k<3;k++) {
        memcpy(&g->index_bbox_min[k],p+root+4*k,4);memcpy(&g->index_bbox_max[k],p+root+12+4*k,4);
        g->world_bbox_min[k]=tvdb__nv_f64(p+560+8*k);g->world_bbox_max[k]=tvdb__nv_f64(p+584+8*k);
        g->voxel_size[k]=tvdb__nv_f64(p+608+8*k);
        g->node_count[k]=tvdb__nv_u32(p+704+4*k);g->tile_count[k]=tvdb__nv_u32(p+716+4*k);
        for (int j=0;j<3;j++) g->map[k*4+j]=tvdb__nv_f64(p+384+8*(k*3+j));
        g->map[k*4+3]=tvdb__nv_f64(p+528+8*k);
    }
    g->node_count[3]=1;g->active_voxel_count=tvdb__nv_u64(p+728);
    if (tvdb__nv_f64(p+552)!=1.0) return TVDB_ERROR_UNSUPPORTED_TRANSFORM;
    if (!g->name) {
        size_t n=0;while(n<256 && p[40+n]) n++;
        g->name=tvdb__nv_alloc(f,n+1);if(!g->name)return TVDB_ERROR_OUT_OF_MEMORY;
        memcpy(g->name,p+40,n);g->name[n]=0;
    }
    return TVDB_OK;
}

tvdb_status_t tvdb_nanovdb_file_open_memory(tvdb_nanovdb_file_t *f,const uint8_t *data,
    size_t size,const tvdb_allocator_t *alloc,tvdb_error_t *err) {
    if (!f || !data || size<16 || (alloc && (!alloc->malloc_fn || !alloc->free_fn))) return TVDB_ERROR_INVALID_ARGUMENT;
    memset(f,0,sizeof(*f));f->file_size=size;if(alloc)f->alloc=*alloc;
    /* NanoVDB wire buffers are native little endian. Accessors operate on
     * aligned uint32 words; copy grid bytes to allocator-aligned storage. */
    const uint32_t one=1;
    if (*(const uint8_t *)&one!=1) return TVDB_ERROR_UNSUPPORTED_VERSION;
    size_t cursor=0;tvdb_status_t status=TVDB_OK;
    while (cursor<size) {
        const uint8_t *p=data+cursor;size_t remaining=size-cursor;
        if (remaining<16) { status=TVDB_ERROR_INVALID_DATA;break; }
        uint64_t magic=tvdb__nv_u64(p);
        int raw=magic==TVDB_NANOVDB_MAGIC_GRID || (magic==TVDB_NANOVDB_MAGIC_NUMB &&
            remaining>=736 && (tvdb__nv_u32(p+8)>>21)!=32 && (tvdb__nv_u32(p+16)>>21)==32);
        if (raw) {
            if (remaining<736) { status=TVDB_ERROR_INVALID_DATA;break; }
            uint64_t bytes=tvdb__nv_u64(p+32);
            if (bytes<736 || bytes>remaining) { status=TVDB_ERROR_INVALID_DATA;break; }
            if (!tvdb__nv_append(f,1)) { status=TVDB_ERROR_OUT_OF_MEMORY;break; }
            tvdb_nanovdb_grid_t *g=&f->grids[f->num_grids-1];g->size=bytes;
            g->data=tvdb__nv_alloc(f,(size_t)bytes);if(!g->data){status=TVDB_ERROR_OUT_OF_MEMORY;break;}
            g->owns_data=1;memcpy(g->data,p,(size_t)bytes);
            status=tvdb__nv_metadata(f,g);if(status!=TVDB_OK)break;
            cursor+=(size_t)bytes;continue;
        }
        if (magic!=TVDB_NANOVDB_MAGIC_FILE && magic!=TVDB_NANOVDB_MAGIC_NUMB) {status=TVDB_ERROR_INVALID_HEADER;break;}
        uint32_t version=tvdb__nv_u32(p+8);uint16_t count,codec;
        memcpy(&count,p+12,2);memcpy(&codec,p+14,2);
        if ((version>>21)!=32 || !count || codec>2) {status=TVDB_ERROR_UNSUPPORTED_VERSION;break;}
        size_t first=f->num_grids;
        if (!tvdb__nv_append(f,count)) {status=TVDB_ERROR_OUT_OF_MEMORY;break;}
        f->version=version;f->codec=codec;cursor+=16;
        /* Metadata's encoded sizes temporarily occupy active_voxel_count;
         * parsing the actual GridData restores that field below. */
        for (uint32_t i=0;i<count;i++) {
            if (size-cursor<176) {status=TVDB_ERROR_INVALID_DATA;break;}
            p=data+cursor;tvdb_nanovdb_grid_t *g=&f->grids[first+i];
            g->size=tvdb__nv_u64(p);g->active_voxel_count=tvdb__nv_u64(p+8);
            uint32_t namesize=tvdb__nv_u32(p+136);cursor+=176;
            if (g->size<736 || g->size>SIZE_MAX || namesize>size-cursor || !namesize) {status=TVDB_ERROR_INVALID_DATA;break;}
            const uint8_t *end=memchr(data+cursor,0,namesize);
            if (!end) {status=TVDB_ERROR_INVALID_DATA;break;}
            size_t n=(size_t)(end-(data+cursor));g->name=tvdb__nv_alloc(f,n+1);
            if(!g->name){status=TVDB_ERROR_OUT_OF_MEMORY;break;}
            memcpy(g->name,data+cursor,n);g->name[n]=0;cursor+=namesize;
        }
        if(status!=TVDB_OK)break;
        for(uint32_t i=0;i<count;i++) {
            tvdb_nanovdb_grid_t *g=&f->grids[first+i];size_t encoded=(size_t)g->active_voxel_count;
            if(encoded>size-cursor || (!codec && encoded!=g->size)){status=TVDB_ERROR_INVALID_DATA;break;}
            g->data=tvdb__nv_alloc(f,(size_t)g->size);if(!g->data){status=TVDB_ERROR_OUT_OF_MEMORY;break;}
            g->owns_data=1;
            if(!codec)memcpy(g->data,data+cursor,(size_t)g->size);
            else {
                size_t read=0,written=0;
                while(written<g->size) {
                    if(encoded-read<8){status=TVDB_ERROR_INVALID_DATA;break;}
                    uint64_t n=tvdb__nv_u64(data+cursor+read);read+=8;
                    if(n>encoded-read){status=TVDB_ERROR_INVALID_DATA;break;}
                    size_t output=(size_t)g->size-written;
                    if(codec==2 && output>1073741824u)output=1073741824u;
                    if(!tvdb__nnvdb_decompress(g->data+written,output,data+cursor+read,(size_t)n,(tvdb_nanovdb_codec_t)codec)) {
                        status=TVDB_ERROR_DECOMPRESSION_FAILED;break;
                    }
                    written+=output;read+=(size_t)n;
                }
                if(status==TVDB_OK && read!=encoded)status=TVDB_ERROR_INVALID_DATA;
                if(status!=TVDB_OK)break;
            }
            cursor+=encoded;status=tvdb__nv_metadata(f,g);if(status!=TVDB_OK)break;
        }
        if(status!=TVDB_OK)break;
    }
    if(status==TVDB_OK && f->num_grids>UINT16_MAX)status=TVDB_ERROR_UNSUPPORTED_VERSION;
    f->grid_count=(uint16_t)f->num_grids;
    if(status!=TVDB_OK){
        /* Release everything decoded so far: the caller has no valid handle to
         * close on this path, so anything left attached to `f` would leak. */
        tvdb__nv_release_grids(f);
        tvdb__nnvdb_set_error(err,status,"Invalid, truncated, unsupported or over-budget NanoVDB input");
    }
    return status;
}

tvdb_status_t tvdb_nanovdb_file_open(tvdb_nanovdb_file_t *f,const char *path,
    const tvdb_allocator_t *alloc,tvdb_error_t *err) {
    if(!f || !path || (alloc && (!alloc->malloc_fn || !alloc->free_fn)))return TVDB_ERROR_INVALID_ARGUMENT;
    memset(f,0,sizeof(*f));if(alloc)f->alloc=*alloc;
#if !defined(TVDB_NO_MMAP) && !defined(_WIN32)
    int fd=open(path,O_RDONLY);
    if(fd>=0) {
        struct stat st;
        if(fstat(fd,&st)==0 && st.st_size>0 && (uint64_t)st.st_size<=SIZE_MAX) {
            void *mapped=mmap(NULL,(size_t)st.st_size,PROT_READ,MAP_PRIVATE,fd,0);
            if(mapped!=MAP_FAILED) {
                close(fd);
                tvdb_status_t result=tvdb_nanovdb_file_open_memory(f,mapped,(size_t)st.st_size,alloc,err);
                /* Grid copies own their bytes; the source mapping need not
                 * outlive parsing, including on partial-allocation failure. */
                munmap(mapped,(size_t)st.st_size);return result;
            }
        }
        close(fd);
    }
#endif
    FILE *fp=fopen(path,"rb");if(!fp)return TVDB_ERROR_IO;
    if(fseek(fp,0,SEEK_END)){fclose(fp);return TVDB_ERROR_IO;}
    long size=ftell(fp);if(size<=0 || fseek(fp,0,SEEK_SET)){fclose(fp);return TVDB_ERROR_IO;}
    uint8_t *data=tvdb__nv_alloc(f,(size_t)size);if(!data){fclose(fp);return TVDB_ERROR_OUT_OF_MEMORY;}
    if(fread(data,1,(size_t)size,fp)!=(size_t)size){tvdb__nv_free(f,data,(size_t)size);fclose(fp);return TVDB_ERROR_IO;}
    fclose(fp);
    tvdb_status_t result=tvdb_nanovdb_file_open_memory(f,data,(size_t)size,alloc,err);
    tvdb__nv_free(f,data,(size_t)size);return result;
}

void tvdb_nanovdb_file_close(tvdb_nanovdb_file_t *f) {
    if(!f)return;
    tvdb__nv_release_grids(f);
    if(f->buffer)tvdb__nv_free(f,f->buffer,(size_t)f->file_size);
#if !defined(TVDB_NO_MMAP) && !defined(_WIN32)
    if(f->mmap_data)munmap((void *)f->mmap_data,(size_t)f->file_size);
#endif
    memset(f,0,sizeof(*f));
}
