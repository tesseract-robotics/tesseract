#include "tvdb_memory.h"
#include <string.h> // For memset

void tvdb_arena_init(tvdb_arena_allocator_t* arena, void* buffer, size_t buffer_size) {
    arena->buffer = buffer;
    arena->buffer_size = buffer_size;
    arena->current_offset = 0;
}

void tvdb_arena_destroy(tvdb_arena_allocator_t* arena) {
    if (arena->buffer) {
        free(arena->buffer); // Assuming buffer was allocated with malloc
        arena->buffer = NULL;
        arena->buffer_size = 0;
        arena->current_offset = 0;
    }
}

void* tvdb_arena_alloc(tvdb_arena_allocator_t* arena, size_t size) {
    size_t alignment = 8; // Align to 8-byte boundary
    size_t aligned_offset = (arena->current_offset + alignment - 1) & ~(alignment - 1);
    size_t required_size = size;

    if (aligned_offset + required_size > arena->buffer_size) {
        return NULL; // Not enough space
    }

    void* ptr = (char*)arena->buffer + aligned_offset;
    arena->current_offset = aligned_offset + required_size;
    
    // Zero-initialize the allocated memory
    memset(ptr, 0, size);
    
    return ptr;
}

void tvdb_arena_reset(tvdb_arena_allocator_t* arena) {
    arena->current_offset = 0;
}
