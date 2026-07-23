#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>  // For size_t

typedef struct
{
  void* buffer;
  size_t buffer_size;
  size_t current_offset;
} tvdb_arena_allocator_t;

// Initialize an arena allocator with a pre-allocated buffer.
// `buffer` should point to a memory block of `buffer_size` bytes.
// The allocator assumes ownership of the buffer pointer and will free it on destroy.
void tvdb_arena_init(tvdb_arena_allocator_t* arena, void* buffer, size_t buffer_size);

// Destroy the arena allocator and free its buffer.
void tvdb_arena_destroy(tvdb_arena_allocator_t* arena);

// Allocate memory from the arena. Returns NULL if allocation fails.
// Basic alignment is handled to align to 8-byte boundaries.
void* tvdb_arena_alloc(tvdb_arena_allocator_t* arena, size_t size);

// Reset the arena allocator, making all previously allocated memory available again.
// This is useful for reusing the same arena for multiple operations within a scope.
void tvdb_arena_reset(tvdb_arena_allocator_t* arena);
