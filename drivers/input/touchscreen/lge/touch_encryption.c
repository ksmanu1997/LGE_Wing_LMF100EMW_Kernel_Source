/*
 * touch_encryption.c
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

#include <linux/random.h>
#include <linux/vmalloc.h>
#include <touch_core.h>
#include <touch_common.h>

/* debug */
static char log_buf[255];
void debug_print_byte(u8 *arr) {
	int i = 0;
	int ret = 0;

	memset(log_buf, 0, sizeof(log_buf));

	for (i = 0; i < BYTE_ARR_SIZE; i++) {
		ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "%4d,", arr[i]);

		if (i % 32 == 32 - 1) {
			ret = 0;
			TOUCH_I("byte : %s\n", log_buf);
			memset(log_buf, 0, sizeof(log_buf));
		}
	}

	if (ret != 0) {
		TOUCH_I("byte : %s\n", log_buf);
	}
}

/* BASE64 API */
void myBase64Decode(char encode_arr[], int encode_arr_size, u8 decode_arr[]) {
	//touch_base64_decode , from_Base64_to_byte
	int value_1 = 0;
	int value_2 = 0;
	int value_3 = 0;
	int remain_arr_size = 0;

	int i = 0;
	int index = 0;
	int total_size = 0;
	int offset = 0;
	u8 decode_buf_arr[BYTE_ARR_SIZE] = {0, };

	for (i = 0; i < encode_arr_size; i++) {
		if(encode_arr[i] == '=') {
			break;
		}
	}
	remain_arr_size = i; // 'i'th last character or the valid size of encode_arr

	for (i = 0; i + 2 < BYTE_ARR_SIZE && remain_arr_size >= 4 && index + 3 < encode_arr_size; i = i + 3) {

		value_1 = (base64_decode_table[(int)encode_arr[index]] & 0x3F) << 2 | (base64_decode_table[(int)encode_arr[index + 1]] & 0x30) >> 4;
		value_2 = (base64_decode_table[(int)encode_arr[index + 1]] & 0xF) << 4 | (base64_decode_table[(int)encode_arr[index + 2]] & 0x3C) >> 2;
		value_3 = (base64_decode_table[(int)encode_arr[index + 2]] & 0x3) << 6 | base64_decode_table[(int)encode_arr[index + 3]];

		decode_buf_arr[i] = (u8) value_1;
		decode_buf_arr[i+1] = (u8) value_2;
		decode_buf_arr[i+2] = (u8) value_3;

		remain_arr_size = remain_arr_size - 4; //use 3 plain_arr
		index = index + 4;
		total_size = total_size + 3;
	}

	if (remain_arr_size == 3 && i + 1 < BYTE_ARR_SIZE && index + 2 < encode_arr_size) {
		value_1 = (base64_decode_table[(int)encode_arr[index]] & 0x3F) << 2 | (base64_decode_table[(int)encode_arr[index + 1]] & 0x30) >> 4;
		value_2 = (base64_decode_table[(int)encode_arr[index + 1]] & 0xF) << 4 | (base64_decode_table[(int)encode_arr[index + 2]] & 0x3C) >> 2;
		decode_buf_arr[i] = (u8) value_1;
		decode_buf_arr[i+1] = (u8) value_2;
		total_size = total_size + 2;
	} else if (remain_arr_size == 2  && i < BYTE_ARR_SIZE && index + 1 < encode_arr_size) {
		value_1 = (base64_decode_table[(int)encode_arr[index]] & 0x3F) << 2 | (base64_decode_table[(int)encode_arr[index + 1]] & 0x30) >> 4;
		decode_buf_arr[i] = (u8) value_1;
		total_size = total_size + 1;
	} else if (remain_arr_size == 1 /* && i < BYTE_ARR_SIZE */) { //no case
		//no case
	}

	offset = BYTE_ARR_SIZE - total_size;
	for (i = 0; i < total_size && offset + i < BYTE_ARR_SIZE; i++) {
		decode_arr[offset + i] = decode_buf_arr[i];
	}
}

void myBase64Encode(u8 plain_arr[], int plain_arr_size, char encode_arr[]) {
	//touch_base64_encode , from_byte_to_base64
	int value_1 = 0;
	int value_2 = 0;
	int value_3 = 0;
	int value_4 = 0;
	int remain_arr_size = 0;

	int i = 0;
	int plain_index = 0;
	char padding_char = '=';

	remain_arr_size = plain_arr_size;

	for (i = 0; i + 3 < BASE64_ENCODING_SIZE && remain_arr_size >= 3 && plain_index + 2 < plain_arr_size; i = i + 4) {

		value_1 = (plain_arr[plain_index] & 0xFC) >> 2;
		value_2 = (plain_arr[plain_index] & 0x3) << 4;
		value_2 = value_2 | ((plain_arr[plain_index + 1] & 0xF0) >> 4);
		value_3 = (plain_arr[plain_index + 1] & 0x0F) << 2;
		value_3 = value_3 | ((plain_arr[plain_index + 2] & 0xC0) >> 6);
		value_4 = plain_arr[plain_index + 2] & 0x3F;

		if (value_1 < 64 && value_2 < 64 && value_3 < 64 && value_4 < 64) {
			encode_arr[i] = base64_encode_table[value_1];
			encode_arr[i+1] = base64_encode_table[value_2];
			encode_arr[i+2] = base64_encode_table[value_3];
			encode_arr[i+3] = base64_encode_table[value_4];

			remain_arr_size = remain_arr_size - 3; //use 3 plain_arr
			plain_index = plain_index + 3;
		}
	}

	if (remain_arr_size == 2 && i + 3 < BASE64_ENCODING_SIZE && plain_index + 1 < plain_arr_size) {
		value_1 = (plain_arr[plain_index] & 0xFC) >> 2;
		value_2 = (plain_arr[plain_index] & 0x3) << 4;
		value_2 = value_2 | ((plain_arr[plain_index + 1] & 0xF0) >> 4);
		value_3 = (plain_arr[plain_index + 1] & 0x0F) << 2;

		if (value_1 < 64 && value_2 < 64 && value_3 < 64) {
			encode_arr[i] = base64_encode_table[value_1];
			encode_arr[i+1] = base64_encode_table[value_2];
			encode_arr[i+2] = base64_encode_table[value_3];
		}
		encode_arr[i+3] = padding_char;
	} else if (remain_arr_size == 1 && i + 3 < BASE64_ENCODING_SIZE && plain_index < plain_arr_size) {
		value_1 = (plain_arr[plain_index] & 0xFC) >> 2;
		value_2 = (plain_arr[plain_index] & 0x3) << 4;

		if (value_1 < 64 && value_2 < 64) {
			encode_arr[i] = base64_encode_table[value_1];
			encode_arr[i+1] = base64_encode_table[value_2];
		}
		encode_arr[i+2] = padding_char;
		encode_arr[i+3] = padding_char;
	}
}

/* BIG interger API */
int unsigned_value(u8 value)
{
	if (value < 0)
		return 256 + value;
	else
		return value;
}
void deepCopy(u8 source[], u8 dest[])
{
	int i = 0;
	for (i = 0; i< BYTE_ARR_SIZE; i++) {
		dest[i] = source[i];
	}
}
int shift_arr_size(int index, int first_bit, int default_arr_size)
{
	int size = default_arr_size;
	if (index != 0 && index < (8 - first_bit))
		size = size + 1;
	return size;
}

void bigInteger_sub(u8 byte_result_buf[], int index, u8 **bit_shift_sub_arr, int rotation, int shift_arr_size)
{
	int i = 0;
	int carry = 0;
	for (i = shift_arr_size - 1; i >= 0 ; i--) {
		if (unsigned_value(byte_result_buf[index + i]) - carry >= unsigned_value(bit_shift_sub_arr[rotation][i])) {
			byte_result_buf[index + i]  = (u8) (unsigned_value(byte_result_buf[index + i]) - carry - unsigned_value(bit_shift_sub_arr[rotation][i]));
			carry = 0;
		} else {
			byte_result_buf[index + i]  = (u8) (256 + unsigned_value(byte_result_buf[index + i]) - carry - unsigned_value(bit_shift_sub_arr[rotation][i]));
			carry = 1;
		}
	}

	if (carry == 1) {
		byte_result_buf[index - 1] = (u8) (byte_result_buf[index - 1] - 1);
	}
}

bool able_to_divide(u8 byte_result_buf[], int index, u8 **bit_shift_sub_arr, int rotation, int shift_arr_size)
{
	int i = 0;
	if (index + shift_arr_size - 1 >= BYTE_ARR_SIZE * 2) //size over
		return false;

	if (unsigned_value(byte_result_buf[index - 1]) > 0) {
		return true;
	}

	for (i = 0; i < shift_arr_size ; i++) {
		if (unsigned_value(byte_result_buf[index + i]) > unsigned_value(bit_shift_sub_arr[rotation][i])) {
			return true;
		} else if (unsigned_value(byte_result_buf[index + i]) < unsigned_value(bit_shift_sub_arr[rotation][i])) {
			return false;
		}
	}
	return true;
}

void generate_each_shift_arr(u8 **bit_shift_sub_arr, u8 compact_mod[], int sub_arr_size, int mod_first_index, int first_bit)
{

	int chekc_first_size_change = 0;
	int pre_arr_size = sub_arr_size + 1;
	int current_arr_size = 0;
	int j = 0;
	int index = 0;

	//index 0
	for (j = 0; j < sub_arr_size; j++) {
		bit_shift_sub_arr[0][j] = compact_mod[j];
	}

	for (index = 1; index < 8; index++) {
		current_arr_size = shift_arr_size(index, first_bit, sub_arr_size);
		if(pre_arr_size != current_arr_size && chekc_first_size_change == 0) {
			chekc_first_size_change = 1;
		}
		for (j = current_arr_size + chekc_first_size_change - 1; j >= 0; j--) {
			if ( j == 0)
				compact_mod[j] = (u8)((compact_mod[j] >> 1) & 0x7F);
			else
				compact_mod[j] = (u8)(((compact_mod[j] >> 1) & 0x7F) | ((compact_mod[j - 1] & 0x01) << 7));
		}
		if (chekc_first_size_change == 1) {
			for (j = 0; j < current_arr_size; j++) {
				compact_mod[j] = compact_mod[j+1];
			}
			chekc_first_size_change = 0;
		}
		for (j = 0; j < current_arr_size; j++) {
			bit_shift_sub_arr[index][j] = compact_mod[j];
		}
		pre_arr_size = current_arr_size;
	}
}

void bigInteger_mod(u8 byte_result_buf[], u8 mod[])  //remainder
{
	int i = 0;
	int j = 0;
	int mod_first_index = 0;
	int byte_result_buf_first_index = 0;
	int first_bit = 0;
	int compare_bit = 0x80; // 1000 0000
	int sub_arr_size = 0;
	int rotation = 0;
	u8 *compact_mod;
	u8 **bit_shift_sub_arr;

	//search first index
	for (i = 0; i < BYTE_ARR_SIZE; i++) {
		if (mod[i] != 0) {
			mod_first_index = i;
			break;
		}
		if (i == BYTE_ARR_SIZE - 1) //mod is 0.
			return;
	}
	sub_arr_size = BYTE_ARR_SIZE - mod_first_index;

	//search first bit
	for (i = 0; i < 8; i++) {
		if ((mod[mod_first_index] & compare_bit) != 0) {
			first_bit = i;
			break;
		}
		compare_bit = compare_bit >> 1;
	}

	//vzalloc
	compact_mod = (u8*)vzalloc(sizeof(u8) * (sub_arr_size + 1)); //+1, need buf
	bit_shift_sub_arr = (u8**)vzalloc(sizeof(u8*) * 8);

	if (compact_mod == NULL || bit_shift_sub_arr == NULL) {
		TOUCH_E("%s: allocate fail!", __func__);
		goto EXIT_SE;
	}
	for (i = 0; i < 8; i++) {
		bit_shift_sub_arr[i] = (u8*)vzalloc(sizeof(u8) * (sub_arr_size + 1)); //+1, need buf
		if (bit_shift_sub_arr[i] == NULL) {
			TOUCH_E("%s: bit_shift_sub_arr[%d] fail!", __func__, i);
			goto EXIT;
		}
	}

	for (i = 0; i < BYTE_ARR_SIZE - mod_first_index; i++) {
		compact_mod[i] = mod[i + mod_first_index];
	}

	generate_each_shift_arr(bit_shift_sub_arr, compact_mod, sub_arr_size, mod_first_index, first_bit);

	//search first index where byte_result
	for (i = 0; i < BYTE_ARR_SIZE * 2; i++) {
		if (byte_result_buf[i] != 0) {
			byte_result_buf_first_index = i;
			break;
		}
	}

	rotation = (8 - first_bit) % 8;

	for (i = byte_result_buf_first_index; i <= BYTE_ARR_SIZE * 2 - sub_arr_size; i++) {
		for (j = 0; j < 8 ; j++) {
			if (able_to_divide(byte_result_buf, i, bit_shift_sub_arr, rotation, shift_arr_size(rotation, first_bit, sub_arr_size))) {
				bigInteger_sub(byte_result_buf, i, bit_shift_sub_arr, rotation, shift_arr_size(rotation, first_bit, sub_arr_size));
			}
			rotation = (rotation + 1) % 8;
		}
	}

EXIT:

	for (i = 0; i < 8; i++) {
		vfree(bit_shift_sub_arr[i]);
	}

EXIT_SE:

	vfree(bit_shift_sub_arr);
	vfree(compact_mod);

}

void bigInteger_multiply_mod(u8 big_a[], u8 big_b[], u8 mod[], u8 result[])
{
	int i = 0;
	int j = 0;
	int index = 0;
	int temp = 0;
	int carry = 0;
	int multiply_result_buf[BYTE_ARR_SIZE * 2] = {0, };
	u8 byte_result_buf[BYTE_ARR_SIZE * 2] = {0, };
	for (i = BYTE_ARR_SIZE - 1; i >= 0; i--) {
		for (j = BYTE_ARR_SIZE - 1; j >= 0; j--) {
			temp = unsigned_value(big_a[i]) * unsigned_value(big_b[j]);
			/* result_buf index = (2 * BYTE_ARR_SIZE - 1) - ((BYTE_ARR_SIZE - 1) - i + (BYTE_ARR_SIZE - 1) - j) */
			index = i + j + 1;
			/* maximum value is smaller integer Max, because arr_size 256 + 10 */
			multiply_result_buf[index] = multiply_result_buf[index] + temp; //need to test
		}
	}
	for (i = BYTE_ARR_SIZE * 2 - 1; i - 1 >= 0; i--) {
		carry = multiply_result_buf[i] / 256;
		multiply_result_buf[i - 1] = multiply_result_buf[i - 1] + carry;
		byte_result_buf[i] = (u8)(multiply_result_buf[i] % 256);
	} //there is margin in buf.
	bigInteger_mod(byte_result_buf, mod);

	for (i = 0; i < BYTE_ARR_SIZE; i++) {
		result[BYTE_ARR_SIZE - 1 - i] = byte_result_buf[BYTE_ARR_SIZE * 2 - 1 - i];
	}
}

void bigInteger_pow_mod(u8 big_arr[], u8 expo[], u8 mod[], u8 result[])
{
	u8 square_buf[BYTE_ARR_SIZE] = {0,};
	u8 result_buf[BYTE_ARR_SIZE] = {0,};
	int check_bit = 1;
	int first_index = 0;
	int i = 0;
	int j = 0;

	result_buf[BYTE_ARR_SIZE - 1] = 1; //init
	square_buf[BYTE_ARR_SIZE - 1] = 1; //init

	bigInteger_multiply_mod(big_arr, square_buf, mod, square_buf);

	for (i = 0; i < BYTE_ARR_SIZE; i++) {
		if (expo[i] != 0) {
			first_index = i;
			break;
		}
	}

	for (i = BYTE_ARR_SIZE - 1; i >= first_index; i--) {
		for (j = 0; j < 8; j++) {
			if ((expo[i] & (check_bit << j)) != 0) {
				bigInteger_multiply_mod(square_buf, result_buf, mod, result_buf);
			}
			bigInteger_multiply_mod(square_buf, square_buf, mod, square_buf);
		}
	}

	deepCopy(result_buf, result);
}

/* RSA API */
void encryptionRSA(u8 plain[], u8 publicExpo[], u8 module[], u8 encryption[])
{
	TOUCH_TRACE();
	bigInteger_pow_mod(plain, publicExpo, module, encryption);
}

void encryption_coordinate(void *ts_void, int id, int mode)
{
	struct touch_core_data *ts = ts_void;
	static int offset = 0;
	char partial_encrpytion_str[PARTIAL_STR_SIZE + 1] = {0,};
	int i = 0, j = 0;
	int read_size = 0;
	int str_encryption_RSA_size = strlen(ts->str_encryption_RSA);
	int plain_coordi[ENCRYP_COEFF_MAT_SIZE] = {0,};
	int encryp_coordi[ENCRYP_COEFF_MAT_SIZE] = {0,};
	int temp = 0;
	int current_offset = 0;

	for (i = 0; i < ENCRYP_COEFF_MAT_SIZE; i++) {
		if (i == 0)
			plain_coordi[i] = ts->tdata[id].x;
		else if (i == 1)
			plain_coordi[i] = ts->tdata[id].y;
		else if (i == 2)
			plain_coordi[i] = ts->tdata[id].pressure;
		else
			plain_coordi[i] = (u8)prandom_u32()%2000; // 2000 is default value.
	}

	for (i = 0; i < ENCRYP_COEFF_MAT_SIZE; i++) {
		temp = 0;
		for (j = 0; j < ENCRYP_COEFF_MAT_SIZE; j++) {
			temp = temp + ts->encryption_matrix[i][j] * plain_coordi[j];
		}
		encryp_coordi[i] = temp;
	}

	if (offset + PARTIAL_STR_SIZE < str_encryption_RSA_size) {
		memcpy(partial_encrpytion_str, ts->str_encryption_RSA + offset, PARTIAL_STR_SIZE);
		current_offset = offset;
		offset = offset + PARTIAL_STR_SIZE;
	} else {
		read_size = str_encryption_RSA_size - offset;
		memcpy(partial_encrpytion_str, ts->str_encryption_RSA + offset, read_size);
		partial_encrpytion_str[read_size] = '\0';
		current_offset = offset;
		offset = 0;
	}

	TOUCH_ENCRYP("[%d,%s][%x,%x,%x,%x,%x][%d,%d,%d]\n",
			current_offset, partial_encrpytion_str,
			encryp_coordi[0], encryp_coordi[1], encryp_coordi[2], encryp_coordi[3], encryp_coordi[4],
			ts->tcount, id, mode);

}
int calculateDet(long matrix[][ENCRYP_COEFF_MAT_SIZE], int size)
{
	int col = 0;
	long det = 0;
	int i = 0, j = 0;

	TOUCH_TRACE();

	if (size == 1) {
		det = matrix[0][0];
		return det;
	} else if ( size == 2 ) {
		det = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
		return det;
	} else { //more than 3
		for (col = 0; col < size ; col++) {

			long subMatrix[ENCRYP_COEFF_MAT_SIZE][ENCRYP_COEFF_MAT_SIZE] = {{0}};

			for (i = 0; i < size - 1; i++) {
				int realJ = 0;
				for (j = 0; j < size; j++) {
					if ( j != col) {
						subMatrix[i][realJ] = matrix[i+1][j];
						realJ++;
					}
				}
			}

			if (col % 2 == 0)
				det = det + matrix[0][col] * calculateDet(subMatrix, size - 1);
			else
				det = det - matrix[0][col] * calculateDet(subMatrix, size - 1);
		}
		return det;
	}
}

void generate_matrix(long matrix[][ENCRYP_COEFF_MAT_SIZE])
{
	int i = 0, j = 0;
	for ( i = 0; i < ENCRYP_COEFF_MAT_SIZE; i++ ) {
		for ( j = 0; j < ENCRYP_COEFF_MAT_SIZE; j++ ) {
			matrix[i][j] = (u8)prandom_u32()%255;
		}
	}
}

void generate_coeff_matrix(void *ts_void)
{
	struct touch_core_data *ts= ts_void;
	long det = 0;
	int cnt = 0;
	int i = 0;
	int encode_first_index = 0;

	u8 *plainDataArray;
	u8 *publicKey_decode_arr;
	u8 *RSA_Module_decode_arr;
	u8 *encrpy_done_arr;

	TOUCH_TRACE();

	plainDataArray = (u8*)vzalloc(sizeof(u8) * BYTE_ARR_SIZE);
	publicKey_decode_arr = (u8*)vzalloc(sizeof(u8) * BYTE_ARR_SIZE);
	RSA_Module_decode_arr = (u8*)vzalloc(sizeof(u8) * BYTE_ARR_SIZE);
	encrpy_done_arr = (u8*)vzalloc(sizeof(u8) * BYTE_ARR_SIZE);

	if (plainDataArray == NULL || publicKey_decode_arr == NULL
			|| RSA_Module_decode_arr == NULL || encrpy_done_arr == NULL) {
		TOUCH_E("%s : allocate fail!\n", __func__);
		goto EXIT;
	}

	while (det == 0 && cnt < 100) {
		generate_matrix(ts->encryption_matrix);
		det = calculateDet(ts->encryption_matrix, ENCRYP_COEFF_MAT_SIZE);
		cnt = cnt + 1;
	}

	if (cnt < 100) {
		TOUCH_I("generate_coeff_matrix sucess!, cnt : %d\n", cnt);
	} else {
		TOUCH_E("generate_coeff_matrix fail!!\n");
	}

	for (i = 0; i < (KEY_SIZE/8) - 1; i++ ) { // because it must be smaller than module.
		if (i < ENCRYP_COEFF_MAT_SIZE * ENCRYP_COEFF_MAT_SIZE)
			plainDataArray[(BYTE_ARR_SIZE - 1) - i] = (u8)ts->encryption_matrix[i/ENCRYP_COEFF_MAT_SIZE][i%ENCRYP_COEFF_MAT_SIZE];
		else
			plainDataArray[(BYTE_ARR_SIZE - 1) - i] = (u8)prandom_u32()%255;
	}

	if (!ts->role.encryption_coordi) {
		TOUCH_I("plainDataArray\n");
		debug_print_byte(plainDataArray); //debug
	}

	// publicExop, modue, base64  => BigInteger
	myBase64Decode(publicKEY, strlen(publicKEY), publicKey_decode_arr); // base64 -> byte
	myBase64Decode(RSA_Module, strlen(RSA_Module), RSA_Module_decode_arr); // base64 -> byte

	// RSA encryption
	encryptionRSA(plainDataArray, publicKey_decode_arr, RSA_Module_decode_arr, encrpy_done_arr); // RSA encryption

	myBase64Encode(encrpy_done_arr, BYTE_ARR_SIZE - encode_first_index, ts->str_encryption_RSA); //byte -> base64

	TOUCH_I("touch_encryption: %s\n", ts->str_encryption_RSA);

EXIT:
	vfree(plainDataArray);
	vfree(publicKey_decode_arr);
	vfree(RSA_Module_decode_arr);
	vfree(encrpy_done_arr);

	return;
}

