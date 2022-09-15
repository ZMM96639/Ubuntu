
#pragma once
#include <iostream>

class Mymemcopy
{
public:
    Mymemcopy() = default;
    ~Mymemcopy() = default;
    char *memcpy(char *dst, char *src, size_t size);
};
