#include "Mymemcopy.hpp"
#include <cstring>

int main()
{
    char s[16] = "aabbcc";
    char d[16] = {0};

    Mymemcopy mymcpy;

    mymcpy.memcpy(s + 2, s, 4);
    printf("memcpy: %s\n", s);
    strcpy(s, "aabbcc");
    mymcpy.memcpy(s + 2, s, 4);
    printf("memcpy: %s\n", s);

    return 0;
}