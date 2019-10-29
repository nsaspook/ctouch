/* 
 * File:   ctouch.h
 * Author: root
 *
 * Created on September 14, 2016, 9:38 AM
 */

#ifndef CTOUCH_H
#define	CTOUCH_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <GenericTypeDefs.h>

#ifdef INTTYPES
#include <stdint.h>
#else
#define INTTYPES
    /*unsigned types*/
    typedef unsigned char uint8_t;
    typedef unsigned int uint16_t;
    typedef unsigned long uint32_t;
    typedef unsigned long long uint64_t;
    /*signed types*/
    typedef signed char int8_t;
    typedef signed int int16_t;
    typedef signed long int32_t;
    typedef signed long long int64_t;
#endif

    const rom uint8_t corners[2][2] = {0x06, 0x5a, 0x72, 0x5a};

#ifdef	__cplusplus
}
#endif

#endif	/* CTOUCH_H */

