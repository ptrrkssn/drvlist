/*
 * utils.c
 *
 * Common util functions
 *
 * Copyright (c) 2026 Peter Eriksson <pen@lysator.liu.se>
 *
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DRVLIST_UTILS_H
#define DRVLIST_UTILS_H 1

#include <stdint.h>
#include <sys/types.h>

extern uint16_t
get_be16(const uint8_t *p);

extern uint16_t
get_le16(const uint8_t *p);

extern uint32_t
get_be32(const uint8_t *p);

extern uint32_t
get_le32(const uint8_t *p);

extern uint64_t
get_be64(const uint8_t *p);

extern uint64_t
get_le64(const uint8_t *p);


extern void
put_be16(uint8_t *p,
	 const uint16_t v);


extern char *
size2str(size_t size);

extern char *
strdupget(const char *s,
	  size_t maxlen);

extern char *
strdupcat(char **old,
	  char *add);

extern int
strtrim(char *str,
	int *len);

extern int
strntrim(char *str,
	 int *len,
	 int max);

#endif
