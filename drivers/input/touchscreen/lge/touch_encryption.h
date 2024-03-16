/*
 * touch_encryption.h
 *
 * Copyright (c) 2015 LGE.
 *
 * author : hoyeon.jang@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* RSA API */
#define KEY_SIZE 		(1024)
static char *RSA_Module = "AK8ZEfsYezwyQDXBgQAN49zR5pmxkQ06AtrSu+Z/jNqYdv6liNDnRu3connUt1ypNsIhrvwj4SlRK2OLQql7fHNEueWqUm2XspT96JPnV8i5ytXqOH4q1qMlFGVBAMIuo2Grug3hqI13B3c6XUAuJ/EucU15doW35jU6VkunHiAt";
static char *publicKEY  = "AQAB";


/* BASE64 API */
#define BASE64_ENCODING_SIZE 		((BYTE_ARR_SIZE*8)/6 + 10) // 1024/6 = 171

static const char padding_char = '=';
static const char base64_encode_table[] = {
	 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
	 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
	 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
	 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
	 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
	 'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
	 'w', 'x', 'y', 'z', '0', '1', '2', '3',
	 '4', '5', '6', '7', '8', '9', '+', '/'
};
static const int base64_decode_table[] = {
	    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  /* 00-0F */
	    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  /* 10-1F */
	    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,62,-1,-1,-1,63,  /* 20-2F */
	    52,53,54,55,56,57,58,59,60,61,-1,-1,-1,-1,-1,-1,  /* 30-3F */
	    -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,  /* 40-4F */
	    15,16,17,18,19,20,21,22,23,24,25,-1,-1,-1,-1,-1,  /* 50-5F */
	    -1,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,  /* 60-6F */
	    41,42,43,44,45,46,47,48,49,50,51,-1,-1,-1,-1,-1,  /* 70-7F */
	    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  /* 80-8F */
	    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  /* 90-9F */
	    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  /* A0-AF */
	    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  /* B0-BF */
	    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  /* C0-CF */
	    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  /* D0-DF */
	    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  /* E0-EF */
	    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1   /* F0-FF */
};

/* matrix API */
#define ENCRYP_COEFF_MAT_SIZE 		(5)
#define BYTE_ARR_SIZE 		(KEY_SIZE/8 + 10)
static long encryption_coeff_mat[ENCRYP_COEFF_MAT_SIZE][ENCRYP_COEFF_MAT_SIZE];

#define PARTIAL_STR_SIZE 		(15)

enum {
	PRESSED = 0,
	RELEASED,
	CANCELED,
};
