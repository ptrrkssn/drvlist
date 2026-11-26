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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "utils.h"


uint64_t
get_be(const uint8_t *p,
       uint8_t len) {
    uint64_t v = 0;

    while (len-- > 0) {
	v <<= 8;
	v |= *p++;
    }

    return v;
}

uint16_t
get_be16(const uint8_t *p)
{
    return (((uint16_t)p[0] << 8) |
	    ((uint16_t)p[1]));
}

uint16_t
get_le16(const uint8_t *p)
{
    return (((uint16_t)p[1] << 8) |
	    ((uint16_t)p[0]));
}

uint32_t
get_be32(const uint8_t *p)
{
    return (((uint32_t)p[0] << 24) |
	    ((uint32_t)p[1] << 16) |
	    ((uint32_t)p[2] << 8) |
	    ((uint32_t)p[3]));
}

uint32_t
get_le32(const uint8_t *p)
{
    return (((uint32_t)p[3] << 24) |
	    ((uint32_t)p[2] << 16) |
	    ((uint32_t)p[1] << 8) |
	    ((uint32_t)p[0]));
}

uint64_t
get_be64(const uint8_t *p)
{
    return (((uint64_t)p[0] << 56) |
	    ((uint64_t)p[1] << 48) |
	    ((uint64_t)p[2] << 40) |
	    ((uint64_t)p[3] << 32) |
	    ((uint64_t)p[4] << 24) |
	    ((uint64_t)p[5] << 16) |
	    ((uint64_t)p[6] << 8) |
	    ((uint64_t)p[7]));
}
	    
	    uint64_t
get_le64(const uint8_t *p)
{
    return (((uint64_t)p[7] << 56) |
	    ((uint64_t)p[6] << 48) |
	    ((uint64_t)p[5] << 40) |
	    ((uint64_t)p[4] << 32) |
	    ((uint64_t)p[3] << 24) |
	    ((uint64_t)p[2] << 16) |
	    ((uint64_t)p[1] << 8) |
	    ((uint64_t)p[0]));
}

	    
void
put_be16(uint8_t *p,
	 const uint16_t v) {
    p[0] = (v>>8);
    p[1] = (v&0xFF);
}




char *
size2str(size_t size) {
    char buf[256];
    double ds = size;
    
    if (size < 9000) {
	sprintf(buf, "%lu", size);
	return strdup(buf);
    }

    ds /= 1000;
    if (ds < 9000) {
	sprintf(buf, "%.0fK", ds);
	return strdup(buf);
    }
    
    ds /= 1000;
    if (ds < 9000) {
	sprintf(buf, "%.0fM", ds);
	return strdup(buf);
    }
    
    ds /= 1000;
    if (ds < 9000) {
	sprintf(buf, "%.0fG", ds);
	return strdup(buf);
    }
	
    
    ds /= 1000;
    if (ds < 9000) {
	sprintf(buf, "%.0fT", ds);
	return strdup(buf);
    }
	
    
    ds /= 1000;
    sprintf(buf, "%.0fP", ds);
    return strdup(buf);
}



/*
 * Duplicate a string, stripping leading and trailing spaces
 * and nonprintable characters
 */
char *
strdupget(const char *s,
	  size_t maxlen) {
    int i;
    char *r;
    

    
    /* Skip leading spaces */
    while (*s && (isspace(*s) || !isprint(*s)) && maxlen > 0) {
	++s;
	--maxlen;
    }

    /* Duplicate the string */
    r = strndup(s, maxlen);
    if (!r)
	return NULL;
    
    /* Strip trailing spaces or nonprintable characters */
    for (i = strlen(r)-1; i >= 0 && (isspace(r[i]) || !isprint(r[i]) || r[i] == '_'); --i)
	r[i] = '\0';

    for (i = 0; r[i]; i++) {
	if (!isprint(r[i]))
	    r[i] = '_';
    }
    return r;
}


char *
strdupcat(char **old,
	  char *add) {
    size_t olen = 0;
    size_t alen = 0;
    
    if (*old) {
	if (strcmp(*old, add) == 0)
	    return 0;
	
	olen += strlen(*old);
    }
    
    if (olen > 0)
	++alen;
    alen += strlen(add);

    if (*old && strcmp(*old, add) > 0) {
	char *new = malloc(olen+alen+1);
	if (!new)
	    return NULL;
	strcpy(new, add);
	strcat(new, ",");
	strcat(new, *old);
	free(*old);
	*old = new;
    } else {
	*old = realloc(*old, olen+alen+1);
	if (!*old)
	    return NULL;
	
	if (olen > 0)
	    (*old)[olen++] = ',';
	
	strcpy((*old)+olen, add);
    }
    
    return *old;
}

int
strtrim(char *str,
	int *len) {
    int i, j, n;

    
    if (!str)
	return 0;

    /* Remove leading whitespace */
    for (i = 0; str[i] && isspace(str[i]); i++)
	;
    if (i > 0) {
	for (j = 0; str[i]; j++, i++)
	    str[j] = str[i];
	str[j] = '\0';
    }

    /* Remove trailing whitespace */
    n = strlen(str);
    while (n > 0 && isspace(str[n-1]))
	--n;
    str[n] = '\0';
    if (len && n > *len)
	*len = n;
    return n;
}


int
strntrim(char *str,
	 int *len,
	 int max) {
    int rlen = strtrim(str, len);

    if (max == 0)
	return rlen;
    
    if (rlen+2 > max) {
	str[max-2] = '.';
	str[max-1] = '.';
	str[max] = '\0';
	rlen = max;
	if (len)
	    *len = rlen;
    }

    return rlen;
}

