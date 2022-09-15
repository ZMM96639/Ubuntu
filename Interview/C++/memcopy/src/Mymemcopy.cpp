#include "Mymemcopy.hpp"

// using mycast = static_cast<const char*>; error(why)
// typedef static_cast<const char *> mycast; error(why)

char *Mymemcopy::memcpy(char *dst, char *src, size_t size)
{
    if (dst == nullptr || src == nullptr)
    {
        return nullptr;
    }

    char *psrc{nullptr};
    char *pdst{nullptr};

    if ((src < dst) && dst < src + size)
    {
        psrc = src + size - 1;
        pdst = dst + size - 1;
        while (size--)
        {
            *pdst-- = *psrc--;
        }
    }
    else
    {
        psrc = src;
        pdst = dst;
        while (size--)
        {
            *pdst++ = *psrc++;
        }
    }
    return pdst;
}