#pragma once

#include <cstdlib>

#ifndef MI_MALLOC_VERSION
#define MI_MALLOC_VERSION 210
#endif

static inline void *mi_malloc_aligned(std::size_t bytes, std::size_t alignment)
{
    void *ptr = nullptr;
    if (alignment < sizeof(void *))
    {
        alignment = sizeof(void *);
    }
    if ((alignment & (alignment - 1)) != 0)
    {
        std::size_t pow2 = sizeof(void *);
        while (pow2 < alignment)
        {
            pow2 <<= 1;
        }
        alignment = pow2;
    }
    if (posix_memalign(&ptr, alignment, bytes) != 0)
    {
        return nullptr;
    }
    return ptr;
}

static inline void mi_free_aligned(void *ptr, std::size_t)
{
    std::free(ptr);
}

