
#pragma once
#ifndef _UTIL_H_
#define _UTIL_H_

#include <string.h>



//#define MIN(a,b) (((a)<(b))?(a):(b))
//#define MAX(a,b) (((a)>(b))?(a):(b))


#ifndef CONCAT2
#define CONCAT2(var1, var2) var1##var2
#endif
#ifndef CONCAT3
#define CONCAT3(var1, var2, var3) var1##var2##var3
#endif

#ifndef STRINGIFY
#define STRINGIFY(var) #var
#endif


inline static void* fast_copy(void* dt, const void* sc, size_t data_len)
{
    #if defined(__CROSSWORKS_ARM) || defined(__SES_ARM) || defined(__SES_RISCV)
        return memcpy_fast(dt, sc, data_len);
    #else
        return memcpy(dt, sc, data_len);
    #endif
}


#endif