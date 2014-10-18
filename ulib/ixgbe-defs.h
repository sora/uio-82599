/**
  UIO-IXGBE - User-space library for Intel 10Gigabit Ethernet adapters
  Copyright (C) 2009 Qualcomm Inc. All rights reserved.
  Written by Max Krasnyansky <maxk@qualcommm.com>

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
 
  1. Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
  
  3. Neither the name of the QUALCOMM Incorporated nor the
  names of any contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY QUALCOMM AND ANY OTHER CONTRIBUTORS 
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL QUALCOMM 
  AND CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED 
  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT(INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
  This file is partially based on the original ixgbe-1.6.4 driver
  for FreeBSD.

  Copyright (c) 2001-2008, Intel Corporation 
  All rights reserved.
  
  Redistribution and use in source and binary forms, with or without 
  modification, are permitted provided that the following conditions are met:
  
   1. Redistributions of source code must retain the above copyright notice, 
      this list of conditions and the following disclaimer.
  
   2. Redistributions in binary form must reproduce the above copyright 
      notice, this list of conditions and the following disclaimer in the 
      documentation and/or other materials provided with the distribution.
  
   3. Neither the name of the Intel Corporation nor the names of its 
      contributors may be used to endorse or promote products derived from 
      this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _IXGBE_DEFS_H_
#define _IXGBE_DEFS_H_

#include <stdint.h>
#include <endian.h>
#include <unistd.h>
#include <sys/io.h>

#if __BYTE_ORDER__ == __BIG_ENDIAN

#define cpu_to_le16(x) bswap_16(x)
#define cpu_to_le32(x) bswap_32(x)
#define cpu_to_le64(x) bswap_64(x)

#define le16_to_cpu(x) bswap_16(x)
#define le32_to_cpu(x) bswap_32(x)
#define le64_to_cpu(x) bswap_64(x)

#else

#define cpu_to_le16(x) (x)
#define cpu_to_le32(x) (x)
#define cpu_to_le64(x) (x)

#define le16_to_cpu(x) (x)
#define le32_to_cpu(x) (x)
#define le64_to_cpu(x) (x)

#endif

#ifndef unlikely
#define unlikely(x)     __builtin_expect(!!(x), 0)
#endif

/* 
   read{w,l} and write{w,l} - read/write from/to MMIO space.

   These should work on all platforms because the space is mmaped. 
   In other words mmap itself would have failed if plaftorm does 
   not support normal MMIO.

   Little endian byte order is assumed.
*/

static inline uint8_t readb(const volatile void *addr)
{
	return *(volatile uint8_t *) addr;
}
static inline uint16_t readw(const volatile void *addr)
{
	return cpu_to_le16( *(volatile uint16_t *) addr );
}
static inline uint32_t readl(const volatile void *addr)
{
	return cpu_to_le32( *(volatile uint32_t *) addr );
}
static inline void writeb(uint8_t b, volatile void *addr)
{
	*(volatile uint8_t *) addr = b;
}
static inline void writew(uint16_t b, volatile void *addr)
{
	*(volatile uint16_t *) addr = cpu_to_le16(b);
}
static inline void writel(uint32_t b, volatile void *addr)
{
	*(volatile uint32_t *) addr = cpu_to_le32(b);
}

#if defined(__x86_64__) || defined(__i386__) || defined(__amd64__)

#define mem_barrier_rw() asm volatile("mfence":::"memory")
#define mem_barrier_r()  asm volatile("lfence":::"memory")
#define mem_barrier_w()  asm volatile("sfence" ::: "memory")

#define mb()  asm volatile("mfence" ::: "memory")
#define wmb() asm volatile("sfence" ::: "memory")
#define rmb() asm volatile("lfence" ::: "memory")

#else

#define mem_barrier_rw() asm volatile("" ::: "memory")
#define mem_barrier_r()  asm volatile("" ::: "memory")
#define mem_barrier_w()  asm volatile("" ::: "memory")

#define mb()  asm volatile("" ::: "memory")
#define rmb() asm volatile("" ::: "memory")
#define wmb() asm volatile("" ::: "memory")

#endif

typedef uint8_t         u8;
typedef int8_t          s8;
typedef uint16_t        u16;
typedef uint32_t        u32;
typedef int32_t         s32;
typedef uint64_t        u64;
typedef uint8_t         bool;

/* only works for sizes that are powers of 2 */
#define IXGBE_ROUNDUP(i, size) ((i) = (((i) + (size) - 1) & ~((size) - 1)))

#define IXGBE_WRITE_REG(a, reg, value) writel((value), a + reg)
#define IXGBE_READ_REG(a, reg)         readl(a + reg)

#define IXGBE_WRITE_FLUSH(a) IXGBE_READ_REG(a, IXGBE_STATUS)

#endif /* _IXGBE_DEFS_H_ */
