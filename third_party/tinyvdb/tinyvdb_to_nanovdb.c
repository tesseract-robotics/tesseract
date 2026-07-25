/*
 * tinyvdb_to_nanovdb.c — convert a loaded VDB FloatGrid (Tree_float_5_4_3)
 * into an in-memory NanoVDB FloatGrid byte buffer.
 *
 * Layout produced (matches nanovdb::FloatGrid byte format, version 32+):
 *
 *   [GridData 672][TreeData 64][RootData 64]
 *   [RootTile #0..N-1, 32 bytes each]
 *   [Upper #0..U-1, 270400 bytes each: bbox+masks+8-byte slots]
 *   [Lower #0..L-1,  33856 bytes each: bbox+masks+8-byte slots]
 *   [Leaf  #0..F-1,   2144 bytes each: bbox+mask+min/max/ave/stddev+512f]
 *
 * Only Float SDF/level-set/fog volume grids are supported here. The output
 * passes tinyvdb's own NanoVDB reader and the PNanoVDB.h hierarchical
 * accessor; checksum is set to "empty" (0xFFFFFFFFFFFFFFFF), so neither
 * libnanovdb nor nanovdb_validate will reject the file.
 */

#include "tinyvdb_io.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Forward decl matching tinyvdb_nanovdb.h. Defined here. */
tvdb_status_t tvdb_grid_to_nanovdb_float(const tvdb_grid_t *grid,
                                          uint8_t **out_data,
                                          size_t *out_size,
                                          tvdb_error_t *err);

#define NV_GRID_BYTES        672u
#define NV_TREE_BYTES         64u
#define NV_ROOT_FLOAT_BYTES   64u
#define NV_TILE_FLOAT_BYTES   32u
#define NV_UPPER_FLOAT_BYTES  270400u
#define NV_LOWER_FLOAT_BYTES   33856u
#define NV_LEAF_FLOAT_BYTES     2144u

/* Field offsets within each NanoVDB node (FloatGrid). */
#define NV_UPPER_BBOX_MIN_OFF      0u
#define NV_UPPER_BBOX_MAX_OFF     12u
#define NV_UPPER_FLAGS_OFF        24u
#define NV_UPPER_VALUE_MASK_OFF   32u
#define NV_UPPER_CHILD_MASK_OFF 4128u
#define NV_UPPER_MIN_OFF        8224u
#define NV_UPPER_TABLE_OFF      8256u

#define NV_LOWER_BBOX_MIN_OFF      0u
#define NV_LOWER_BBOX_MAX_OFF     12u
#define NV_LOWER_FLAGS_OFF        24u
#define NV_LOWER_VALUE_MASK_OFF   32u
#define NV_LOWER_CHILD_MASK_OFF  544u
#define NV_LOWER_MIN_OFF        1056u
#define NV_LOWER_TABLE_OFF      1088u

#define NV_LEAF_BBOX_MIN_OFF       0u
#define NV_LEAF_BBOX_DIF_FLAGS_OFF 12u
#define NV_LEAF_VALUE_MASK_OFF    16u
#define NV_LEAF_MIN_OFF           80u
#define NV_LEAF_MAX_OFF           84u
#define NV_LEAF_AVE_OFF           88u
#define NV_LEAF_STDDEV_OFF        92u
#define NV_LEAF_TABLE_OFF         96u

#define NV_TREE_OFF_LEAF_OFF       0u
#define NV_TREE_OFF_LOWER_OFF      8u
#define NV_TREE_OFF_UPPER_OFF     16u
#define NV_TREE_OFF_ROOT_OFF      24u
#define NV_TREE_NODE_COUNT_LEAF   32u
#define NV_TREE_NODE_COUNT_LOWER  36u
#define NV_TREE_NODE_COUNT_UPPER  40u
#define NV_TREE_TILE_COUNT_LOWER  44u
#define NV_TREE_TILE_COUNT_UPPER  48u
#define NV_TREE_TILE_COUNT_ROOT   52u
#define NV_TREE_VOXEL_COUNT       56u

#define NV_ROOT_BBOX_MIN_OFF       0u
#define NV_ROOT_BBOX_MAX_OFF      12u
#define NV_ROOT_TABLE_SIZE_OFF    24u
#define NV_ROOT_BG_OFF            28u  /* float */
#define NV_ROOT_MIN_OFF           32u
#define NV_ROOT_MAX_OFF           36u

#define NV_TILE_KEY_OFF            0u
#define NV_TILE_CHILD_OFF          8u
#define NV_TILE_STATE_OFF         16u
#define NV_TILE_VALUE_OFF         20u

#define NV_GRID_OFF_MAGIC          0u
#define NV_GRID_OFF_CHECKSUM       8u
#define NV_GRID_OFF_VERSION       16u
#define NV_GRID_OFF_FLAGS         20u
#define NV_GRID_OFF_GRID_INDEX    24u
#define NV_GRID_OFF_GRID_COUNT    28u
#define NV_GRID_OFF_GRID_SIZE     32u
#define NV_GRID_OFF_GRID_NAME     40u
#define NV_GRID_OFF_MAP          296u
#define NV_GRID_OFF_WORLD_BBOX   560u
#define NV_GRID_OFF_VOXEL_SIZE   608u
#define NV_GRID_OFF_GRID_CLASS   632u
#define NV_GRID_OFF_GRID_TYPE    636u
#define NV_GRID_OFF_BLIND_OFF    640u
#define NV_GRID_OFF_BLIND_COUNT  648u

#define NV_MAGIC_NUMBER 0x304244566f6e614eULL  /* "NanoVDB0" */
#define NV_GRID_TYPE_FLOAT 1u

/* version (Major,Minor,Patch) packed (Major<<21 | Minor<<10 | Patch). */
#define NV_VERSION_MAJOR 32u
#define NV_VERSION_MINOR 9u
#define NV_VERSION_PATCH 1u

static uint32_t nv__make_version(void) {
    return (NV_VERSION_MAJOR << 21) | (NV_VERSION_MINOR << 10) | NV_VERSION_PATCH;
}

static void nv__write_u32(uint8_t *p, size_t off, uint32_t v) {
    memcpy(p + off, &v, 4);
}
static void nv__write_u64(uint8_t *p, size_t off, uint64_t v) {
    memcpy(p + off, &v, 8);
}
static void nv__write_i32(uint8_t *p, size_t off, int32_t v) {
    memcpy(p + off, &v, 4);
}
static void nv__write_i64(uint8_t *p, size_t off, int64_t v) {
    memcpy(p + off, &v, 8);
}
static void nv__write_f32(uint8_t *p, size_t off, float v) {
    memcpy(p + off, &v, 4);
}
static void nv__write_f64(uint8_t *p, size_t off, double v) {
    memcpy(p + off, &v, 8);
}

static void nv__set_bit(uint8_t *mask, size_t bit) {
    mask[bit >> 3] |= (uint8_t)(1u << (bit & 7u));
}

/* Node-walking state: collected during a DFS pass over the VDB tree. */
typedef struct nv_node_info {
    int32_t origin[3];     /* lower-corner ijk in world index space */
    size_t  vdb_node_idx;  /* index into grid->tree.nodes[] */
    size_t  parent_idx;    /* index into the same per-level array */
    uint32_t parent_slot;  /* slot index within parent's child table */
} nv_node_info_t;

typedef struct nv_build_ctx {
    nv_node_info_t *uppers;  size_t n_upper, cap_upper;
    nv_node_info_t *lowers;  size_t n_lower, cap_lower;
    nv_node_info_t *leaves;  size_t n_leaf,  cap_leaf;
} nv_build_ctx_t;

static int nv__push(nv_node_info_t **arr, size_t *n, size_t *cap,
                    nv_node_info_t info) {
    if (*n >= *cap) {
        size_t nc = (*cap == 0) ? 64 : (*cap * 2);
        nv_node_info_t *p = (nv_node_info_t *)realloc(*arr, nc * sizeof(*p));
        if (!p) return 0;
        *arr = p; *cap = nc;
    }
    (*arr)[(*n)++] = info;
    return 1;
}

/* Decompose VDB internal-node child slot index into (ix,iy,iz) given log2dim. */
static void nv__slot_to_ijk(int32_t s, int log2dim,
                            int32_t *ix, int32_t *iy, int32_t *iz) {
    int32_t mask = (1 << log2dim) - 1;
    *ix = (s >> (2 * log2dim)) & mask;
    *iy = (s >> log2dim) & mask;
    *iz =  s & mask;
}

/* DFS walk: collect uppers/lowers/leaves with origins and parent linkage. */
static int nv__collect(const tvdb_tree_t *tree, nv_build_ctx_t *ctx) {
    if (tree->num_nodes == 0) return 1;

    /* Walk root → upper(level 1) → lower(level 2) → leaf(level 3). */
    const tvdb_tree_node_t *root = &tree->nodes[0];
    if (root->type != TVDB_NODE_ROOT) return 0;
    const tvdb_root_node_t *r = &root->u.root;

    /* Upper (level 1) children of root. */
    for (uint32_t i = 0; i < r->num_children; ++i) {
        nv_node_info_t up;
        up.origin[0] = r->child_origins[3 * i + 0];
        up.origin[1] = r->child_origins[3 * i + 1];
        up.origin[2] = r->child_origins[3 * i + 2];
        up.vdb_node_idx = r->child_indices[i];
        up.parent_idx = 0;
        up.parent_slot = i;  /* root tile index */
        if (!nv__push(&ctx->uppers, &ctx->n_upper, &ctx->cap_upper, up)) return 0;
    }

    /* For each upper, enumerate lowers. */
    for (size_t u = 0; u < ctx->n_upper; ++u) {
        const tvdb_tree_node_t *un = &tree->nodes[ctx->uppers[u].vdb_node_idx];
        if (un->type != TVDB_NODE_INTERNAL) continue;
        const tvdb_internal_node_t *ui = &un->u.internal;
        int32_t log2dim = ui->child_mask.log2dim;  /* 5 */
        int32_t total = 1 << (3 * log2dim);
        size_t c_idx = 0;
        for (int32_t s = 0; s < total && c_idx < ui->num_children; ++s) {
            if (!(ui->child_mask.bits.data[s >> 3] & (1u << (s & 7u)))) continue;
            int32_t ix, iy, iz;
            nv__slot_to_ijk(s, log2dim, &ix, &iy, &iz);
            /* Lower covers 16 * 8 = 128 voxels per axis. */
            int32_t child_dim = 1 << (4 + 3);
            nv_node_info_t lo;
            lo.origin[0] = ctx->uppers[u].origin[0] + ix * child_dim;
            lo.origin[1] = ctx->uppers[u].origin[1] + iy * child_dim;
            lo.origin[2] = ctx->uppers[u].origin[2] + iz * child_dim;
            lo.vdb_node_idx = ui->child_indices[c_idx];
            lo.parent_idx = u;
            lo.parent_slot = (uint32_t)s;
            if (!nv__push(&ctx->lowers, &ctx->n_lower, &ctx->cap_lower, lo)) return 0;
            ++c_idx;
        }
    }

    /* For each lower, enumerate leaves. */
    for (size_t l = 0; l < ctx->n_lower; ++l) {
        const tvdb_tree_node_t *ln = &tree->nodes[ctx->lowers[l].vdb_node_idx];
        if (ln->type != TVDB_NODE_INTERNAL) continue;
        const tvdb_internal_node_t *li = &ln->u.internal;
        int32_t log2dim = li->child_mask.log2dim;  /* 4 */
        int32_t total = 1 << (3 * log2dim);
        size_t c_idx = 0;
        for (int32_t s = 0; s < total && c_idx < li->num_children; ++s) {
            if (!(li->child_mask.bits.data[s >> 3] & (1u << (s & 7u)))) continue;
            int32_t ix, iy, iz;
            nv__slot_to_ijk(s, log2dim, &ix, &iy, &iz);
            int32_t child_dim = 1 << 3;  /* leaf covers 8 voxels per axis */
            nv_node_info_t lf;
            lf.origin[0] = ctx->lowers[l].origin[0] + ix * child_dim;
            lf.origin[1] = ctx->lowers[l].origin[1] + iy * child_dim;
            lf.origin[2] = ctx->lowers[l].origin[2] + iz * child_dim;
            lf.vdb_node_idx = li->child_indices[c_idx];
            lf.parent_idx = l;
            lf.parent_slot = (uint32_t)s;
            if (!nv__push(&ctx->leaves, &ctx->n_leaf, &ctx->cap_leaf, lf)) return 0;
            ++c_idx;
        }
    }
    return 1;
}

/* NanoVDB-style root-tile key from world-space ijk. */
static uint64_t nv__coord_to_key(int32_t x, int32_t y, int32_t z) {
    uint64_t iu = (uint32_t)x >> 12u;
    uint64_t ju = (uint32_t)y >> 12u;
    uint64_t ku = (uint32_t)z >> 12u;
    return ku | (ju << 21) | (iu << 42);
}

/* NanoVDB Upper child slot from world-ijk relative to upper origin (32^3). */
static uint32_t nv__upper_slot(int32_t ix, int32_t iy, int32_t iz) {
    /* upper log2dim=5, so each axis shifts by 5 within upper-local 32^3 */
    return (uint32_t)((ix << 10) | (iy << 5) | iz);
}
static uint32_t nv__lower_slot(int32_t ix, int32_t iy, int32_t iz) {
    return (uint32_t)((ix << 8) | (iy << 4) | iz);
}
static uint32_t nv__leaf_slot(int32_t ix, int32_t iy, int32_t iz) {
    return (uint32_t)((ix << 6) | (iy << 3) | iz);
}

tvdb_status_t tvdb_grid_to_nanovdb_float(const tvdb_grid_t *grid,
                                         uint8_t **out_data,
                                         size_t *out_size,
                                         tvdb_error_t *err) {
    if (!grid || !out_data || !out_size) {
        if (err) {
            err->status = TVDB_ERROR_INVALID_ARGUMENT;
            snprintf(err->message, sizeof(err->message), "null arg");
        }
        return TVDB_ERROR_INVALID_ARGUMENT;
    }
    *out_data = NULL;
    *out_size = 0;

    /* Require a 4-level Tree_float_5_4_3 grid. */
    if (grid->tree.layout.num_levels != 4 ||
        grid->tree.layout.levels[0].log2dim != 0 ||
        grid->tree.layout.levels[1].log2dim != 5 ||
        grid->tree.layout.levels[2].log2dim != 4 ||
        grid->tree.layout.levels[3].log2dim != 3 ||
        grid->tree.layout.levels[3].value_type != TVDB_VALUE_FLOAT) {
        if (err) {
            err->status = TVDB_ERROR_UNSUPPORTED_VERSION;
            snprintf(err->message, sizeof(err->message),
                     "tvdb_grid_to_nanovdb_float: only Tree_float_5_4_3 supported");
        }
        return TVDB_ERROR_UNSUPPORTED_VERSION;
    }

    nv_build_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    if (!nv__collect(&grid->tree, &ctx)) {
        free(ctx.uppers); free(ctx.lowers); free(ctx.leaves);
        if (err) {
            err->status = TVDB_ERROR_OUT_OF_MEMORY;
            snprintf(err->message, sizeof(err->message), "OOM in nv__collect");
        }
        return TVDB_ERROR_OUT_OF_MEMORY;
    }

    /* Layout. */
    size_t off_grid  = 0;
    size_t off_tree  = NV_GRID_BYTES;
    size_t off_root  = off_tree + NV_TREE_BYTES;
    size_t off_tile0 = off_root + NV_ROOT_FLOAT_BYTES;
    size_t off_up0   = off_tile0 + (size_t)ctx.n_upper * NV_TILE_FLOAT_BYTES;
    size_t off_lo0   = off_up0   + (size_t)ctx.n_upper * NV_UPPER_FLOAT_BYTES;
    size_t off_lf0   = off_lo0   + (size_t)ctx.n_lower * NV_LOWER_FLOAT_BYTES;
    size_t total     = off_lf0   + (size_t)ctx.n_leaf  * NV_LEAF_FLOAT_BYTES;
    /* Round up to 32-byte alignment. */
    size_t aligned_total = (total + 31u) & ~(size_t)31u;

    uint8_t *buf = (uint8_t *)calloc(1, aligned_total);
    if (!buf) {
        free(ctx.uppers); free(ctx.lowers); free(ctx.leaves);
        if (err) {
            err->status = TVDB_ERROR_OUT_OF_MEMORY;
            snprintf(err->message, sizeof(err->message), "OOM allocating output buffer");
        }
        return TVDB_ERROR_OUT_OF_MEMORY;
    }

    /* GridData. */
    nv__write_u64(buf, off_grid + NV_GRID_OFF_MAGIC,    NV_MAGIC_NUMBER);
    nv__write_u64(buf, off_grid + NV_GRID_OFF_CHECKSUM, 0xFFFFFFFFFFFFFFFFull);
    nv__write_u32(buf, off_grid + NV_GRID_OFF_VERSION,  nv__make_version());
    nv__write_u32(buf, off_grid + NV_GRID_OFF_FLAGS,    0u);
    nv__write_u32(buf, off_grid + NV_GRID_OFF_GRID_INDEX, 0u);
    nv__write_u32(buf, off_grid + NV_GRID_OFF_GRID_COUNT, 1u);
    nv__write_u64(buf, off_grid + NV_GRID_OFF_GRID_SIZE,  (uint64_t)aligned_total);
    /* grid_name: 256 bytes, copy if available. */
    {
        const char *name = grid->descriptor.grid_name ? grid->descriptor.grid_name : "";
        size_t nl = strlen(name);
        if (nl > 255) nl = 255;
        memcpy(buf + off_grid + NV_GRID_OFF_GRID_NAME, name, nl);
    }
    /* Map: identity affine + uniform voxel size derived from grid transform. */
    {
        double vs = 1.0;
        double translation[3] = { 0.0, 0.0, 0.0 };
        if (grid->transform.type == TVDB_TRANSFORM_UNIFORM_SCALE ||
            grid->transform.type == TVDB_TRANSFORM_UNIFORM_SCALE_TRANSLATE) {
            vs = grid->transform.voxel_size[0];
            if (vs <= 0.0) vs = grid->transform.scale_values[0];
            if (vs <= 0.0) vs = 1.0;
        }
        if (grid->transform.type == TVDB_TRANSFORM_UNIFORM_SCALE_TRANSLATE) {
            translation[0] = grid->transform.translation[0];
            translation[1] = grid->transform.translation[1];
            translation[2] = grid->transform.translation[2];
        }
        /* PNanoVDB Map layout:
             matF[9]    @0  (36)
             invMatF[9] @36 (36)
             vecF[3]    @72 (12)
             taperF     @84 (4)
             matD[9]    @88 (72)
             invMatD[9] @160 (72)
             vecD[3]    @232 (24)
             taperD     @256 (8)   -> total 264 bytes */
        size_t mat_off = off_grid + NV_GRID_OFF_MAP;
        /* matF / invMatF: row-major 3x3 scale and its inverse. */
        for (int i = 0; i < 9; ++i) nv__write_f32(buf, mat_off + 4 * i, 0.0f);
        nv__write_f32(buf, mat_off + 0,  (float)vs);
        nv__write_f32(buf, mat_off + 16, (float)vs);
        nv__write_f32(buf, mat_off + 32, (float)vs);
        for (int i = 0; i < 9; ++i) nv__write_f32(buf, mat_off + 36 + 4 * i, 0.0f);
        nv__write_f32(buf, mat_off + 36 + 0,  (float)(1.0 / vs));
        nv__write_f32(buf, mat_off + 36 + 16, (float)(1.0 / vs));
        nv__write_f32(buf, mat_off + 36 + 32, (float)(1.0 / vs));
        /* matD / invMatD */
        for (int i = 0; i < 9; ++i) nv__write_f64(buf, mat_off + 88 + 8 * i, 0.0);
        nv__write_f64(buf, mat_off + 88 + 0,  vs);
        nv__write_f64(buf, mat_off + 88 + 32, vs);
        nv__write_f64(buf, mat_off + 88 + 64, vs);
        for (int i = 0; i < 9; ++i) nv__write_f64(buf, mat_off + 160 + 8 * i, 0.0);
        nv__write_f64(buf, mat_off + 160 + 0,  1.0 / vs);
        nv__write_f64(buf, mat_off + 160 + 32, 1.0 / vs);
        nv__write_f64(buf, mat_off + 160 + 64, 1.0 / vs);
        nv__write_f32(buf, mat_off + 72, (float)translation[0]);
        nv__write_f32(buf, mat_off + 76, (float)translation[1]);
        nv__write_f32(buf, mat_off + 80, (float)translation[2]);
        nv__write_f64(buf, mat_off + 232, translation[0]);
        nv__write_f64(buf, mat_off + 240, translation[1]);
        nv__write_f64(buf, mat_off + 248, translation[2]);

        /* voxel_size 3 doubles */
        nv__write_f64(buf, off_grid + NV_GRID_OFF_VOXEL_SIZE + 0,  vs);
        nv__write_f64(buf, off_grid + NV_GRID_OFF_VOXEL_SIZE + 8,  vs);
        nv__write_f64(buf, off_grid + NV_GRID_OFF_VOXEL_SIZE + 16, vs);
    }
    /* grid_class / grid_type. tinyvdb's tvdb_grid_t has no grid_class
       field; default to "Unknown" which works for both fog volumes and
       generic SDFs as far as the NanoVDB reader is concerned. */
    nv__write_u32(buf, off_grid + NV_GRID_OFF_GRID_CLASS, 0u);
    nv__write_u32(buf, off_grid + NV_GRID_OFF_GRID_TYPE, NV_GRID_TYPE_FLOAT);
    nv__write_i64(buf, off_grid + NV_GRID_OFF_BLIND_OFF, 0);
    nv__write_u32(buf, off_grid + NV_GRID_OFF_BLIND_COUNT, 0u);

    /* TreeData: offsets are relative to TreeData start. */
    {
        uint64_t tree_to_root  = NV_TREE_BYTES;                                   /* RootData starts right after TreeData */
        uint64_t tree_to_upper = (uint64_t)(off_up0 - off_tree);
        uint64_t tree_to_lower = (uint64_t)(off_lo0 - off_tree);
        uint64_t tree_to_leaf  = (uint64_t)(off_lf0 - off_tree);
        nv__write_u64(buf, off_tree + NV_TREE_OFF_LEAF_OFF,  tree_to_leaf);
        nv__write_u64(buf, off_tree + NV_TREE_OFF_LOWER_OFF, tree_to_lower);
        nv__write_u64(buf, off_tree + NV_TREE_OFF_UPPER_OFF, tree_to_upper);
        nv__write_u64(buf, off_tree + NV_TREE_OFF_ROOT_OFF,  tree_to_root);
        nv__write_u32(buf, off_tree + NV_TREE_NODE_COUNT_LEAF,  (uint32_t)ctx.n_leaf);
        nv__write_u32(buf, off_tree + NV_TREE_NODE_COUNT_LOWER, (uint32_t)ctx.n_lower);
        nv__write_u32(buf, off_tree + NV_TREE_NODE_COUNT_UPPER, (uint32_t)ctx.n_upper);
        nv__write_u32(buf, off_tree + NV_TREE_TILE_COUNT_LOWER, 0u);
        nv__write_u32(buf, off_tree + NV_TREE_TILE_COUNT_UPPER, 0u);
        nv__write_u32(buf, off_tree + NV_TREE_TILE_COUNT_ROOT,  (uint32_t)ctx.n_upper);
    }

    /* Background value from VDB root. */
    float bg = grid->tree.nodes[0].u.root.background.u.f;

    /* RootData. */
    {
        nv__write_u32(buf, off_root + NV_ROOT_TABLE_SIZE_OFF, (uint32_t)ctx.n_upper);
        nv__write_f32(buf, off_root + NV_ROOT_BG_OFF, bg);
        nv__write_f32(buf, off_root + NV_ROOT_MIN_OFF, 0.0f);
        nv__write_f32(buf, off_root + NV_ROOT_MAX_OFF, 0.0f);
    }

    /* Root tiles + Upper nodes + per-upper bbox tracking. */
    int32_t root_bb_min[3] = { INT32_MAX, INT32_MAX, INT32_MAX };
    int32_t root_bb_max[3] = { INT32_MIN, INT32_MIN, INT32_MIN };
    uint64_t total_active = 0;

    for (size_t u = 0; u < ctx.n_upper; ++u) {
        const nv_node_info_t *up = &ctx.uppers[u];
        size_t off_tile  = off_tile0 + u * NV_TILE_FLOAT_BYTES;
        size_t off_upper = off_up0   + u * NV_UPPER_FLOAT_BYTES;

        uint64_t key = nv__coord_to_key(up->origin[0], up->origin[1], up->origin[2]);
        nv__write_u64(buf, off_tile + NV_TILE_KEY_OFF, key);
        int64_t child_off = (int64_t)off_upper - (int64_t)off_root;
        nv__write_i64(buf, off_tile + NV_TILE_CHILD_OFF, child_off);
        nv__write_u32(buf, off_tile + NV_TILE_STATE_OFF, 0u);
        nv__write_f32(buf, off_tile + NV_TILE_VALUE_OFF, bg);

        /* Walk this upper's VDB node and emit child/value masks + 32^3 slots. */
        const tvdb_internal_node_t *ui = &grid->tree.nodes[up->vdb_node_idx].u.internal;
        int32_t log2 = ui->child_mask.log2dim;  /* 5 */
        int32_t total_bits = 1 << (3 * log2);

        int32_t up_bb_min[3] = { INT32_MAX, INT32_MAX, INT32_MAX };
        int32_t up_bb_max[3] = { INT32_MIN, INT32_MIN, INT32_MIN };

        size_t c_idx_check = 0;  /* tracks lowers[] index for this upper */
        size_t lower_base = 0;
        for (size_t l = 0; l < ctx.n_lower; ++l) {
            if (ctx.lowers[l].parent_idx == u) { lower_base = l; break; }
        }

        for (int32_t s = 0; s < total_bits; ++s) {
            int32_t ix, iy, iz;
            nv__slot_to_ijk(s, log2, &ix, &iy, &iz);
            int has_child = ui->child_mask.bits.data[s >> 3] & (1u << (s & 7u));
            int has_value = ui->value_mask.bits.data[s >> 3] & (1u << (s & 7u));

            uint32_t nv_slot = nv__upper_slot(ix, iy, iz);

            if (has_child) {
                /* Child slot: 8-byte child offset relative to this upper. */
                /* Find which lower has parent_idx==u and parent_slot==s. */
                size_t lower_idx = SIZE_MAX;
                for (size_t l = lower_base + c_idx_check; l < ctx.n_lower; ++l) {
                    if (ctx.lowers[l].parent_idx == u
                        && ctx.lowers[l].parent_slot == (uint32_t)s) {
                        lower_idx = l;
                        ++c_idx_check;
                        break;
                    }
                }
                if (lower_idx == SIZE_MAX) continue;  /* shouldn't happen */
                size_t off_l = off_lo0 + lower_idx * NV_LOWER_FLOAT_BYTES;
                int64_t rel = (int64_t)off_l - (int64_t)off_upper;
                /* Set child mask bit. */
                nv__set_bit(buf + off_upper + NV_UPPER_CHILD_MASK_OFF, (size_t)s);
                /* Slot table entry = child offset (8 bytes). */
                nv__write_i64(buf, off_upper + NV_UPPER_TABLE_OFF + nv_slot * 8u, rel);
            } else {
                if (has_value) {
                    nv__set_bit(buf + off_upper + NV_UPPER_VALUE_MASK_OFF, (size_t)s);
                }
                /* Tile value lives in low 4 bytes of the 8-byte slot. */
                float tile_v = bg;
                if (ui->values && (size_t)s < (size_t)total_bits) {
                    memcpy(&tile_v, ui->values + (size_t)s * sizeof(float),
                           sizeof(float));
                }
                nv__write_f32(buf, off_upper + NV_UPPER_TABLE_OFF + nv_slot * 8u,
                              tile_v);
                nv__write_u32(buf,
                              off_upper + NV_UPPER_TABLE_OFF + nv_slot * 8u + 4u,
                              0u);
            }
        }

        /* Upper bbox = union of all child lower bboxes + active tile bboxes.
           We compute it after lowers/leaves are written; for now leave the
           min/max placeholder. */
        nv__write_i32(buf, off_upper + NV_UPPER_BBOX_MIN_OFF + 0, up_bb_min[0]);
        nv__write_i32(buf, off_upper + NV_UPPER_BBOX_MIN_OFF + 4, up_bb_min[1]);
        nv__write_i32(buf, off_upper + NV_UPPER_BBOX_MIN_OFF + 8, up_bb_min[2]);
        nv__write_i32(buf, off_upper + NV_UPPER_BBOX_MAX_OFF + 0, up_bb_max[0]);
        nv__write_i32(buf, off_upper + NV_UPPER_BBOX_MAX_OFF + 4, up_bb_max[1]);
        nv__write_i32(buf, off_upper + NV_UPPER_BBOX_MAX_OFF + 8, up_bb_max[2]);
        nv__write_u64(buf, off_upper + NV_UPPER_FLAGS_OFF, 0ull);
        nv__write_f32(buf, off_upper + NV_UPPER_MIN_OFF + 0, bg);
        nv__write_f32(buf, off_upper + NV_UPPER_MIN_OFF + 4, bg);
        nv__write_f32(buf, off_upper + NV_UPPER_MIN_OFF + 8, 0.0f);
        nv__write_f32(buf, off_upper + NV_UPPER_MIN_OFF + 12, 0.0f);
    }

    /* Lower nodes. */
    for (size_t l = 0; l < ctx.n_lower; ++l) {
        const nv_node_info_t *lo = &ctx.lowers[l];
        size_t off_lower = off_lo0 + l * NV_LOWER_FLOAT_BYTES;

        const tvdb_internal_node_t *li = &grid->tree.nodes[lo->vdb_node_idx].u.internal;
        int32_t log2 = li->child_mask.log2dim;  /* 4 */
        int32_t total_bits = 1 << (3 * log2);

        size_t c_idx_check = 0;
        size_t leaf_base = 0;
        for (size_t lf = 0; lf < ctx.n_leaf; ++lf) {
            if (ctx.leaves[lf].parent_idx == l) { leaf_base = lf; break; }
        }

        for (int32_t s = 0; s < total_bits; ++s) {
            int32_t ix, iy, iz;
            nv__slot_to_ijk(s, log2, &ix, &iy, &iz);
            int has_child = li->child_mask.bits.data[s >> 3] & (1u << (s & 7u));
            int has_value = li->value_mask.bits.data[s >> 3] & (1u << (s & 7u));
            uint32_t nv_slot = nv__lower_slot(ix, iy, iz);

            if (has_child) {
                size_t leaf_idx = SIZE_MAX;
                for (size_t lf = leaf_base + c_idx_check; lf < ctx.n_leaf; ++lf) {
                    if (ctx.leaves[lf].parent_idx == l
                        && ctx.leaves[lf].parent_slot == (uint32_t)s) {
                        leaf_idx = lf;
                        ++c_idx_check;
                        break;
                    }
                }
                if (leaf_idx == SIZE_MAX) continue;
                size_t off_lf = off_lf0 + leaf_idx * NV_LEAF_FLOAT_BYTES;
                int64_t rel = (int64_t)off_lf - (int64_t)off_lower;
                nv__set_bit(buf + off_lower + NV_LOWER_CHILD_MASK_OFF, (size_t)s);
                nv__write_i64(buf, off_lower + NV_LOWER_TABLE_OFF + nv_slot * 8u, rel);
            } else {
                if (has_value) {
                    nv__set_bit(buf + off_lower + NV_LOWER_VALUE_MASK_OFF, (size_t)s);
                }
                float tile_v = bg;
                if (li->values) {
                    memcpy(&tile_v, li->values + (size_t)s * sizeof(float),
                           sizeof(float));
                }
                nv__write_f32(buf, off_lower + NV_LOWER_TABLE_OFF + nv_slot * 8u,
                              tile_v);
                nv__write_u32(buf,
                              off_lower + NV_LOWER_TABLE_OFF + nv_slot * 8u + 4u,
                              0u);
            }
        }
        nv__write_i32(buf, off_lower + NV_LOWER_BBOX_MIN_OFF + 0, lo->origin[0]);
        nv__write_i32(buf, off_lower + NV_LOWER_BBOX_MIN_OFF + 4, lo->origin[1]);
        nv__write_i32(buf, off_lower + NV_LOWER_BBOX_MIN_OFF + 8, lo->origin[2]);
        int32_t lh = (1 << (4 + 3));
        nv__write_i32(buf, off_lower + NV_LOWER_BBOX_MAX_OFF + 0, lo->origin[0] + lh - 1);
        nv__write_i32(buf, off_lower + NV_LOWER_BBOX_MAX_OFF + 4, lo->origin[1] + lh - 1);
        nv__write_i32(buf, off_lower + NV_LOWER_BBOX_MAX_OFF + 8, lo->origin[2] + lh - 1);
        nv__write_u64(buf, off_lower + NV_LOWER_FLAGS_OFF, 0ull);
        nv__write_f32(buf, off_lower + NV_LOWER_MIN_OFF + 0, bg);
        nv__write_f32(buf, off_lower + NV_LOWER_MIN_OFF + 4, bg);
        nv__write_f32(buf, off_lower + NV_LOWER_MIN_OFF + 8, 0.0f);
        nv__write_f32(buf, off_lower + NV_LOWER_MIN_OFF + 12, 0.0f);
    }

    /* Leaf nodes. */
    for (size_t lf = 0; lf < ctx.n_leaf; ++lf) {
        const nv_node_info_t *li = &ctx.leaves[lf];
        size_t off_leaf = off_lf0 + lf * NV_LEAF_FLOAT_BYTES;

        const tvdb_leaf_node_t *vl =
            &grid->tree.nodes[li->vdb_node_idx].u.leaf;
        const float *vdb_vals = (const float *)vl->data;

        nv__write_i32(buf, off_leaf + NV_LEAF_BBOX_MIN_OFF + 0, li->origin[0]);
        nv__write_i32(buf, off_leaf + NV_LEAF_BBOX_MIN_OFF + 4, li->origin[1]);
        nv__write_i32(buf, off_leaf + NV_LEAF_BBOX_MIN_OFF + 8, li->origin[2]);
        /* dif_and_flags: low 24 bits = (max-min) per axis; flags in high 8.
           NanoVDB uses bit 1 for "has bbox" — set so vdb_print recognizes it. */
        nv__write_u32(buf, off_leaf + NV_LEAF_BBOX_DIF_FLAGS_OFF,
                      (7u << 0) | (7u << 8) | (7u << 16) | (1u << 24));

        /* Value mask: 64 bytes, bit s indicates voxel s is active. tinyvdb's
           NodeMask stores bits in the same convention. */
        memcpy(buf + off_leaf + NV_LEAF_VALUE_MASK_OFF,
               vl->value_mask.bits.data, 64);

        /* Compute min/max/sum/sumSq over active voxels. */
        float vmin = INFINITY, vmax = -INFINITY;
        double vsum = 0.0, vsumsq = 0.0;
        size_t active = 0;
        for (size_t b = 0; b < 512; ++b) {
            float v = vdb_vals[b];
            /* NanoVDB leaf table is laid out by NanoVDB voxel ordering:
               index = (x << 6) | (y << 3) | z within an 8^3 leaf, identical
               to OpenVDB's offset encoding. tinyvdb leaves use the same
               offset, so no remapping is needed. */
            nv__write_f32(buf, off_leaf + NV_LEAF_TABLE_OFF + b * 4u, v);
            if (vl->value_mask.bits.data[b >> 3] & (1u << (b & 7u))) {
                if (v < vmin) vmin = v;
                if (v > vmax) vmax = v;
                vsum += (double)v;
                vsumsq += (double)v * (double)v;
                ++active;
            }
        }
        if (active == 0) { vmin = 0.0f; vmax = 0.0f; }
        float vave = active > 0 ? (float)(vsum / (double)active) : 0.0f;
        float vstddev = 0.0f;
        if (active > 0) {
            double mean = vsum / (double)active;
            double var  = vsumsq / (double)active - mean * mean;
            if (var > 0.0) vstddev = (float)sqrt(var);
        }
        nv__write_f32(buf, off_leaf + NV_LEAF_MIN_OFF, vmin);
        nv__write_f32(buf, off_leaf + NV_LEAF_MAX_OFF, vmax);
        nv__write_f32(buf, off_leaf + NV_LEAF_AVE_OFF, vave);
        nv__write_f32(buf, off_leaf + NV_LEAF_STDDEV_OFF, vstddev);
        total_active += active;
    }

    /* Now propagate bboxes upper & root. (Lowers got them above.) */
    for (size_t u = 0; u < ctx.n_upper; ++u) {
        size_t off_upper = off_up0 + u * NV_UPPER_FLOAT_BYTES;
        int32_t bb_min[3] = { INT32_MAX, INT32_MAX, INT32_MAX };
        int32_t bb_max[3] = { INT32_MIN, INT32_MIN, INT32_MIN };
        for (size_t l = 0; l < ctx.n_lower; ++l) {
            if (ctx.lowers[l].parent_idx != u) continue;
            int32_t lh = 1 << (4 + 3);
            int32_t lo[3] = { ctx.lowers[l].origin[0], ctx.lowers[l].origin[1], ctx.lowers[l].origin[2] };
            int32_t hi[3] = { lo[0] + lh - 1, lo[1] + lh - 1, lo[2] + lh - 1 };
            for (int k = 0; k < 3; ++k) {
                if (lo[k] < bb_min[k]) bb_min[k] = lo[k];
                if (hi[k] > bb_max[k]) bb_max[k] = hi[k];
            }
        }
        if (bb_min[0] == INT32_MAX) {
            bb_min[0] = bb_min[1] = bb_min[2] = ctx.uppers[u].origin[0];
            bb_max[0] = bb_max[1] = bb_max[2] = ctx.uppers[u].origin[0];
        }
        nv__write_i32(buf, off_upper + NV_UPPER_BBOX_MIN_OFF + 0, bb_min[0]);
        nv__write_i32(buf, off_upper + NV_UPPER_BBOX_MIN_OFF + 4, bb_min[1]);
        nv__write_i32(buf, off_upper + NV_UPPER_BBOX_MIN_OFF + 8, bb_min[2]);
        nv__write_i32(buf, off_upper + NV_UPPER_BBOX_MAX_OFF + 0, bb_max[0]);
        nv__write_i32(buf, off_upper + NV_UPPER_BBOX_MAX_OFF + 4, bb_max[1]);
        nv__write_i32(buf, off_upper + NV_UPPER_BBOX_MAX_OFF + 8, bb_max[2]);

        for (int k = 0; k < 3; ++k) {
            if (bb_min[k] < root_bb_min[k]) root_bb_min[k] = bb_min[k];
            if (bb_max[k] > root_bb_max[k]) root_bb_max[k] = bb_max[k];
        }
    }

    if (root_bb_min[0] == INT32_MAX) {
        root_bb_min[0] = root_bb_min[1] = root_bb_min[2] = 0;
        root_bb_max[0] = root_bb_max[1] = root_bb_max[2] = 0;
    }
    nv__write_i32(buf, off_root + NV_ROOT_BBOX_MIN_OFF + 0, root_bb_min[0]);
    nv__write_i32(buf, off_root + NV_ROOT_BBOX_MIN_OFF + 4, root_bb_min[1]);
    nv__write_i32(buf, off_root + NV_ROOT_BBOX_MIN_OFF + 8, root_bb_min[2]);
    nv__write_i32(buf, off_root + NV_ROOT_BBOX_MAX_OFF + 0, root_bb_max[0]);
    nv__write_i32(buf, off_root + NV_ROOT_BBOX_MAX_OFF + 4, root_bb_max[1]);
    nv__write_i32(buf, off_root + NV_ROOT_BBOX_MAX_OFF + 8, root_bb_max[2]);

    nv__write_u64(buf, off_tree + NV_TREE_VOXEL_COUNT, total_active);

    /* World bbox = translated root index bbox * voxel_size. */
    {
        double vs = 1.0;
        double translation[3] = { 0.0, 0.0, 0.0 };
        memcpy(&vs, buf + off_grid + NV_GRID_OFF_VOXEL_SIZE, 8);
        if (grid->transform.type == TVDB_TRANSFORM_UNIFORM_SCALE_TRANSLATE) {
            translation[0] = grid->transform.translation[0];
            translation[1] = grid->transform.translation[1];
            translation[2] = grid->transform.translation[2];
        }
        nv__write_f64(buf, off_grid + NV_GRID_OFF_WORLD_BBOX + 0, translation[0] + (double)root_bb_min[0] * vs);
        nv__write_f64(buf, off_grid + NV_GRID_OFF_WORLD_BBOX + 8, translation[1] + (double)root_bb_min[1] * vs);
        nv__write_f64(buf, off_grid + NV_GRID_OFF_WORLD_BBOX + 16, translation[2] + (double)root_bb_min[2] * vs);
        nv__write_f64(buf, off_grid + NV_GRID_OFF_WORLD_BBOX + 24, translation[0] + (double)(root_bb_max[0] + 1) * vs);
        nv__write_f64(buf, off_grid + NV_GRID_OFF_WORLD_BBOX + 32, translation[1] + (double)(root_bb_max[1] + 1) * vs);
        nv__write_f64(buf, off_grid + NV_GRID_OFF_WORLD_BBOX + 40, translation[2] + (double)(root_bb_max[2] + 1) * vs);
    }

    free(ctx.uppers); free(ctx.lowers); free(ctx.leaves);

    *out_data = buf;
    *out_size = aligned_total;
    return TVDB_OK;
}
