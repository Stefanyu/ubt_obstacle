#ifndef __M_PRINTF_H
#define __M_PRINTF_H

#include <stdio.h>

#define DEBUG 1

#if DEBUG
    #define m_printf(fmt,args...) \
    do\
    {\
        printf(fmt,##args);\
        printf("\n");\
    }while(0)


    #define m_str_printf(fmt,length) \
    do\
    {\
        if(length > 0) \
        {\
            printf(#fmt"= ");\
            for(int i = 0;i < length;i ++) \
                printf("%02x ",fmt[i]);\
            printf("\n");\
        }\
    }while(0)

#else
    #define m_printf(fmt,args...)
    #define m_str_printf(fmt,length)
#endif

#endif
