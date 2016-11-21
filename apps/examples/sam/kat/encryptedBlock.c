/*******************************************************************************
Copyright (C) 2016 Marvell International Ltd.

This software file (the "File") is owned and distributed by Marvell
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the three
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.

********************************************************************************
Marvell Commercial License Option

If you received this File from Marvell and you have entered into a commercial
license agreement (a "Commercial License") with Marvell, the File is licensed
to you under the terms of the applicable Commercial License.

*******************************************************************************/

#include "encryptedBlock.h"
#include "common.h"
#include "array.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/**
 * operations_t: Structure for operations,
 *  	with all of its relevant information.
 *
 * operationCounter - The number of operations.
 * iv - The iv of each operation .
 * icb - The digest of each operation Respectively.
 * plainText - The plain text of each operation Respectively.
 * cipherText - The cipher text of each operation Respectively.
 * cryptoOffset - The offset to made for each operation.
 * cryptoOffsetIndex - The index of the next empty cryptoOffset.
 */
typedef struct operations_t {
	int operationCounter;
	ArrayComplexPtr iv;
	ArrayComplexPtr icb;
	ArrayComplexPtr plainText;
	ArrayComplexPtr cipherText;
	int* cryptoOffset;
	int cryptoOffsetIndex;
}*Operations;
/**
 * encryptedBlock_t: Structure for encrypted block,
 *  	with all of its relevant information.
 *
 * algorithm - The algorithm type.
 * mode - The mode type.
 * authAlgorithm - The authentication algorithm type.
 * direction - The direction type.
 * name - The name of the encrypted block.
 * key - The key of the encrypted block.
 * authKey - The authentication key of the encrypted block.
 * operations - the operations of the current encrypted block.
 * testCounter - The number of loop test for each operation.
 */
struct encryptedBlock_t {
	EncryptedBlockAlgorithm algorithm;
	EncryptedBlockMode mode;
	EncryptedBlockAuthAlgorithm authAlgorithm;
	EncryptedBlockDirection direction;
	ArrayPtr name;
	ArrayPtr key;
	ArrayPtr authKey;
	Operations operations;
	int testCounter;
};
///////////////////////////////////////////////////////////////////////////////
/**
 * encryptedBlockReset: resets the given encrypted block.
 *
 * @param encryptedBlock - The encrypted block to be reseted
 */
static void encryptedBlockReset(EncryptedBlockPtr encryptedBlock);
/**
 * encryptedBlockAddComplexArray:
 *
 * 		Adds a new data element to the given encrypted block element.
 * 		If the element doesn't exist he creates it.
 *
 * @param type - The type of the element
 * @param encryptedBlock - The encrypted block to add the element to
 * @param elementArray - The element to be added
 * @param elementLen - The element length
 * @return
 * 	ENCRYPTEDBLOCK_NULL_ARGS - If at least one of the parameters is NULL
 *	ENCRYPTEDBLOCK_NOT_VALID_ARGS - If at least one of the arg's is not valid
 *	ENCRYPTEDBLOCK_OUT_OF_MEMORY - If an allocation failed
 *	ENCRYPTEDBLOCK_NOT_ENOUGH_OPERATIONS - If the plain text array is full
 * 	ENCRYPTEDBLOCK_SUCCESS - If the function succeeded to add the element
 */
static EncryptedBlockMessage encryptedBlockAddComplexArray(
		EncryptedBlockType type, EncryptedBlockPtr encryptedBlock,
		unsigned char* elementArray, int elementLen);
/**
 * encryptedBlockAddArray:
 *
 * 		creates a new data element.
 *
 * @param type - The type of the element
 * @param encryptedBlock - The encrypted block to add the element to
 * @param elementArray - The element to be added
 * @param elementLen - The element length
 * @return
 * 	ENCRYPTEDBLOCK_NULL_ARGS - If at least one of the parameters is NULL
 *	ENCRYPTEDBLOCK_NOT_VALID_ARGS - If at least one of the arg's is not valid
 *	ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST - If the element is already exist in the block
 *	ENCRYPTEDBLOCK_OUT_OF_MEMORY - If an allocation failed
 * 	ENCRYPTEDBLOCK_SUCCESS - If the function succeeded to add the element
 */
static EncryptedBlockMessage encryptedBlockAddArray(EncryptedBlockType type,
		EncryptedBlockPtr encryptedBlock, unsigned char* elementArray,
		int elementLen);
/**
 * encryptedBlockGetElement: Gets the wanted element pointer.
 *
 * @param type - The type of the wanted element.
 * @param encryptedBlock - The encryptedBlock to get the element from.
 * @return
 * 	pointer to the wanterd element.
 */
static void* encryptedBlockGetElement(EncryptedBlockType type,
		EncryptedBlockPtr encryptedBlock);

/**
 * encryptedBlockConvert(ELEMNT): Converts the given string to the element type.
 *
 * @param data - The string to get the type from.
 * @return
 * 	element type.
 */
static EncryptedBlockAlgorithm encryptedBlockConvertAlgorithm(char* data);
static EncryptedBlockAuthAlgorithm encryptedBlockConvertAuthAlgorithm(
		char* data);
static EncryptedBlockMode encryptedBlockConvertMode(char* data);
static EncryptedBlockDirection encryptedBlockConvertDirection(char* data);
/**
 * encryptedBlockConvertMessage: Converts array message to encrypted block message.
 *
 * @param message - The array message to be converted
 * 	 @return
 * 		the relevant encrypted block message
 */
static EncryptedBlockMessage encryptedBlockConvertMessage(ArrayMessage message);

static EncryptedBlockMessage copyOperations(Operations src, Operations dst);
///////////////////////////////////////////////////////////////////////////////

EncryptedBlockMessage encryptedBlockCreate(
		EncryptedBlockPtr* outputEncryptedBlock) {
	if (!outputEncryptedBlock) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	*outputEncryptedBlock = malloc(sizeof(*(*outputEncryptedBlock)));
	if (*outputEncryptedBlock == NULL) {
		return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
	}
	(*outputEncryptedBlock)->operations = malloc(
			sizeof(*(*outputEncryptedBlock)->operations));
	if ((*outputEncryptedBlock)->operations == NULL) {
		free(*outputEncryptedBlock);
		return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
	}
	encryptedBlockReset(*outputEncryptedBlock);
	return ENCRYPTEDBLOCK_SUCCESS;
}
static void encryptedBlockReset(EncryptedBlockPtr encryptedBlock) {
	if (encryptedBlock == NULL) {
		return;
	}
	encryptedBlock->name = NULL;
	encryptedBlock->key = NULL;
	encryptedBlock->authKey = NULL;
	encryptedBlock->algorithm = ALGORITHM_NULL;
	encryptedBlock->mode = MODE_NULL;
	encryptedBlock->authAlgorithm = AUTH_NULL;
	encryptedBlock->direction = DIRECTION_NULL;
	encryptedBlock->testCounter = 0;
	encryptedBlock->operations->operationCounter = 1;
	encryptedBlock->operations->iv = NULL;
	encryptedBlock->operations->icb = NULL;
	encryptedBlock->operations->plainText = NULL;
	encryptedBlock->operations->cipherText = NULL;
	encryptedBlock->operations->cryptoOffset = NULL;
	encryptedBlock->operations->cryptoOffsetIndex = 0;
}
EncryptedBlockMessage encryptedBlockCopy(EncryptedBlockPtr srcBlock,
		EncryptedBlockPtr* copyBlock) {
	if (!srcBlock || !copyBlock) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	EncryptedBlockMessage message = encryptedBlockCreate(copyBlock);
	if (message != ENCRYPTEDBLOCK_SUCCESS) {
		return message;
	}
	(*copyBlock)->testCounter = srcBlock->testCounter;
	(*copyBlock)->algorithm = srcBlock->algorithm;
	(*copyBlock)->authAlgorithm = srcBlock->authAlgorithm;
	(*copyBlock)->mode = srcBlock->mode;
	(*copyBlock)->direction = srcBlock->direction;

	if (srcBlock->name) {
		(*copyBlock)->name = arrayCopy(srcBlock->name);
		if (!(*copyBlock)->name) {
			encryptedBlockDestroy(*copyBlock);
			*copyBlock = NULL;
			return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
		}
	}
	if (srcBlock->key) {
		(*copyBlock)->key = arrayCopy(srcBlock->key);
		if (!(*copyBlock)->key) {
			encryptedBlockDestroy(*copyBlock);
			*copyBlock = NULL;
			return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
		}
	}
	if (srcBlock->authKey) {
		(*copyBlock)->authKey = arrayCopy(srcBlock->authKey);
		if (!(*copyBlock)->authKey) {
			encryptedBlockDestroy(*copyBlock);
			*copyBlock = NULL;
			return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
		}
	}
	if (copyOperations(srcBlock->operations, (*copyBlock)->operations)
			!= ENCRYPTEDBLOCK_SUCCESS) {
		encryptedBlockDestroy(*copyBlock);
		*copyBlock = NULL;
		return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
	}
	return ENCRYPTEDBLOCK_SUCCESS;
}
void encryptedBlockDestroy(EncryptedBlockPtr encryptedBlock) {
	if (encryptedBlock == NULL) {
		return;
	}
	arrayComplexDestroy(encryptedBlock->operations->iv);
	arrayComplexDestroy(encryptedBlock->operations->icb);
	arrayComplexDestroy(encryptedBlock->operations->plainText);
	arrayComplexDestroy(encryptedBlock->operations->cipherText);
	free(encryptedBlock->operations->cryptoOffset);
	free(encryptedBlock->operations);
	arrayDestroy(encryptedBlock->name);
	arrayDestroy(encryptedBlock->key);
	arrayDestroy(encryptedBlock->authKey);
	free(encryptedBlock);
}
EncryptedBlockMessage encryptedBlockAddElement(EncryptedBlockPtr encryptedBlock,
		EncryptedBlockType type, unsigned char* elementArray, int elementLen) {
	if (!encryptedBlock
			|| (!elementArray
					&& (type != CRYPTO_OFFSET && type != TESTCOUNTER
							&& type != OPERATIONCOUNTER))) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	switch (type) {
	case NAME:
	case KEY:
	case AUTH_KEY:
		return encryptedBlockAddArray(type, encryptedBlock, elementArray,
				elementLen);
		break;
	case TESTCOUNTER:
		encryptedBlock->testCounter = elementLen;
		break;
	case ALGORITHM:
		if (encryptedBlock->algorithm
				&& encryptedBlock->algorithm != ALGORITHM_INVALID) {
			return ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST;
		}
		encryptedBlock->algorithm = encryptedBlockConvertAlgorithm(
				(char*) elementArray);
		if (encryptedBlock->algorithm == ALGORITHM_INVALID) {
			return ENCRYPTEDBLOCK_NOT_VALID_ARGS;
		}
		break;
	case AUTH_ALGORITHM:
		if (encryptedBlock->authAlgorithm
				&& encryptedBlock->authAlgorithm != AUTH_INVALID) {
			return ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST;
		}
		encryptedBlock->authAlgorithm = encryptedBlockConvertAuthAlgorithm(
				(char*) elementArray);
		if (encryptedBlock->authAlgorithm == AUTH_INVALID) {
			return ENCRYPTEDBLOCK_NOT_VALID_ARGS;
		}
		break;
	case MODE:
		if (encryptedBlock->mode && encryptedBlock->mode != MODE_INVALID) {
			return ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST;
		}
		encryptedBlock->mode = encryptedBlockConvertMode((char*) elementArray);
		if (encryptedBlock->mode == MODE_INVALID) {
			return ENCRYPTEDBLOCK_NOT_VALID_ARGS;
		}
		break;
	case DIRECTION:
		encryptedBlock->direction = encryptedBlockConvertDirection(
				(char*) elementArray);
		if (encryptedBlock->direction == DIRECTION_INVALID) {
			return ENCRYPTEDBLOCK_NOT_VALID_ARGS;
		}
		break;
	case OPERATIONCOUNTER:
		if (encryptedBlock->operations->plainText
				|| encryptedBlock->operations->cipherText
				|| encryptedBlock->operations->iv
				|| encryptedBlock->operations->icb
				|| encryptedBlock->operations->cryptoOffset) {
			return ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST;
		}
		encryptedBlock->operations->operationCounter = elementLen;
		break;
	case IV:
	case ICB:
	case PLAINTEXT:
	case CIPHERTEXT:
		return encryptedBlockAddComplexArray(type, encryptedBlock, elementArray,
				elementLen);
		break;
	case CRYPTO_OFFSET:
		if (!encryptedBlock->operations->cryptoOffset) {
			encryptedBlock->operations->cryptoOffset = malloc(
					sizeof(*(encryptedBlock->operations->cryptoOffset))
							* encryptedBlock->operations->operationCounter);
		}
		return encryptedBlockSetCryptoOffset(encryptedBlock, elementLen);
		break;
	default:
		return ENCRYPTEDBLOCK_NOT_VALID_ARGS;
	}
	return ENCRYPTEDBLOCK_SUCCESS;
}

static EncryptedBlockMessage encryptedBlockAddComplexArray(
		EncryptedBlockType type, EncryptedBlockPtr encryptedBlock,
		unsigned char* elementArray, int elementLen) {
	ArrayComplexPtr* dst = encryptedBlockGetElement(type, encryptedBlock);
	if(!dst){
		return ENCRYPTEDBLOCK_NOT_VALID_ARGS;
	}
	if (!*dst) {
		*dst = arrayComplexCreate(encryptedBlock->operations->operationCounter);
		if (!*dst) {
			return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
		}
	}
	return encryptedBlockConvertMessage(
			arrayComplexAddData(*dst, elementArray, elementLen));
}
static EncryptedBlockMessage encryptedBlockAddArray(EncryptedBlockType type,
		EncryptedBlockPtr encryptedBlock, unsigned char* elementArray,
		int elementLen) {
	ArrayPtr* dst = encryptedBlockGetElement(type, encryptedBlock);
	if(!dst){
		return ENCRYPTEDBLOCK_NOT_VALID_ARGS;
	}
	if (*dst) {
		return ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST;
	}
	*dst = arrayCreate(elementArray, elementLen);
	if (!*dst) {
		return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
	}
	return ENCRYPTEDBLOCK_SUCCESS;
}
static void* encryptedBlockGetElement(EncryptedBlockType type,
		EncryptedBlockPtr encryptedBlock) {
	switch (type) {
	case NAME:
		return &encryptedBlock->name;
		break;
	case KEY:
		return &encryptedBlock->key;
		break;
	case AUTH_KEY:
		return &encryptedBlock->authKey;
		break;
	case IV:
		return &encryptedBlock->operations->iv;
		break;
	case ICB:
		return &encryptedBlock->operations->icb;
		break;
	case PLAINTEXT:
		return &encryptedBlock->operations->plainText;
		break;
	case CIPHERTEXT:
		return &encryptedBlock->operations->cipherText;
		break;
	default:
		return NULL;
	}
	return NULL;
}
EncryptedBlockMessage encryptedBlockSetTestCounter(
		EncryptedBlockPtr encryptedBlock, int testCounter) {
	if (!encryptedBlock) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	if (testCounter < 0) {
		return ENCRYPTEDBLOCK_NEGATIVE_COUNTER;
	}
	encryptedBlock->testCounter = testCounter;
	return ENCRYPTEDBLOCK_SUCCESS;
}
EncryptedBlockMessage encryptedBlockSetCryptoOffset(
		EncryptedBlockPtr encryptedBlock, int off) {
	if (!encryptedBlock) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	int index = encryptedBlock->operations->cryptoOffsetIndex;
	if (off < 0) {
		return ENCRYPTEDBLOCK_NEGATIVE_COUNTER;
	}
	if (index >= encryptedBlock->operations->operationCounter) {
		return ENCRYPTEDBLOCK_NOT_ENOUGH_OPERATIONS;
	}
	encryptedBlock->operations->cryptoOffset[index] = off;
	encryptedBlock->operations->cryptoOffsetIndex++;
	return ENCRYPTEDBLOCK_SUCCESS;
}
static EncryptedBlockAlgorithm encryptedBlockConvertAlgorithm(char* data) {
	if (!data) {
		return ALGORITHM_NULL;
	}
	if (strcmp(data, "DES") == 0) {
		return ALGORITHM_DES;
	}
	if (strcmp(data, "3DES") == 0) {
		return ALGORITHM_3DES;
	}
	if (strcmp(data, "AES") == 0) {
		return ALGORITHM_AES;
	}
	if (strcmp(data, "NULL") == 0) {
		return ALGORITHM_NULL;
	}
	printf("Syntax error in Algorithm: %s is unknown Algorithm\n", data);
	return ALGORITHM_INVALID;
}
static EncryptedBlockMode encryptedBlockConvertMode(char* data) {
	if (!data) {
		return MODE_NULL;
	}
	if (strcmp(data, "ECB") == 0) {
		return MODE_ECB;
	}
	if (strcmp(data, "CBC") == 0) {
		return MODE_CBC;
	}
	if (strcmp(data, "CTR") == 0) {
		return MODE_CTR;
	}
	if (strcmp(data, "GCM") == 0) {
		return MODE_GCM;
	}
	if (strcmp(data, "GMAC") == 0) {
		return MODE_GMAC;
	}
	if (strcmp(data, "NULL") == 0) {
		return MODE_NULL;
	}
	printf("Syntax error in Mode: %s is unknown Mode\n", data);
	return MODE_INVALID;
}
static EncryptedBlockAuthAlgorithm encryptedBlockConvertAuthAlgorithm(
		char* data) {
	if (!data) {
		return AUTH_NULL;
	}
	if (strcmp(data, "MD5") == 0) {
		return AUTH_MD5;
	}
	if (strcmp(data, "SHA1") == 0) {
		return AUTH_SHA1;
	}
	if (strcmp(data, "SHA224") == 0) {
		return AUTH_SHA224;
	}
	if (strcmp(data, "SHA256") == 0) {
		return AUTH_SHA256;
	}
	if (strcmp(data, "SHA384") == 0) {
		return AUTH_SHA384;
	}
	if (strcmp(data, "SHA512") == 0) {
		return AUTH_SHA512;
	}
	if (strcmp(data, "NULL") == 0) {
		return AUTH_NULL;
	}
	printf("Syntax error in Authalgorithm: %s is unknown Authalgorithm\n",
			data);
	return AUTH_INVALID;
}
static EncryptedBlockDirection encryptedBlockConvertDirection(char* data) {
	if (!data) {
		return DIRECTION_NULL;
	}
	if (strcmp(data, "encryption") == 0) {
		return ENCRYPTION;
	}
	if (strcmp(data, "decryption") == 0) {
		return DECRYPTION;
	}
	if (strcmp(data, "NULL") == 0) {
		return DIRECTION_NULL;
	}
	printf("Syntax error in Direction: %s is unknown Direction\n", data);
	return DIRECTION_INVALID;
}
EncryptedBlockMessage encryptedBlockGetName(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char* outputArray) {
	if (!encryptedBlock || !outputArray) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	if (!encryptedBlock->name) {
		return ENCRYPTEDBLOCK_NO_ELEMENT;
	}
	return encryptedBlockConvertMessage(
			arrayGetData(encryptedBlock->name, outputArray, size));
}
EncryptedBlockMessage encryptedBlockGetKey(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char* outputArray) {
	if (!encryptedBlock || !outputArray) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	if (!encryptedBlock->key) {
		return ENCRYPTEDBLOCK_NO_ELEMENT;
	}
	return encryptedBlockConvertMessage(
			arrayGetData(encryptedBlock->key, outputArray, size));
}
EncryptedBlockMessage encryptedBlockGetAuthKey(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char* outputArray) {
	if (!encryptedBlock || !outputArray) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	if (!encryptedBlock->authKey) {
		return ENCRYPTEDBLOCK_NO_ELEMENT;
	}
	return encryptedBlockConvertMessage(
			arrayGetData(encryptedBlock->authKey, outputArray, size));
}
EncryptedBlockMessage encryptedBlockGetIv(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char* outputArray, int index) {
	if (!encryptedBlock || !outputArray) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	if (!encryptedBlock->operations->iv) {
		return ENCRYPTEDBLOCK_NO_ELEMENT;
	}
	return encryptedBlockConvertMessage(
			arrayComplexGetData(encryptedBlock->operations->iv, index,
					outputArray, size));
}
EncryptedBlockMessage encryptedBlockGetIcb(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char* outputArray, int index) {
	if (!encryptedBlock || !outputArray) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	if (!encryptedBlock->operations->icb) {
		return ENCRYPTEDBLOCK_NO_ELEMENT;
	}
	return encryptedBlockConvertMessage(
			arrayComplexGetData(encryptedBlock->operations->icb, index,
					outputArray, size));
}
EncryptedBlockMessage encryptedBlockGetPlainText(
		EncryptedBlockPtr encryptedBlock, int size, unsigned char* outputArray,
		int index) {
	if (!encryptedBlock || !outputArray) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	if (!encryptedBlock->operations->plainText) {
		return ENCRYPTEDBLOCK_NO_ELEMENT;
	}
	return encryptedBlockConvertMessage(
			arrayComplexGetData(encryptedBlock->operations->plainText, index,
					outputArray, size));
}
EncryptedBlockMessage encryptedBlockGetCipherText(
		EncryptedBlockPtr encryptedBlock, int size, unsigned char* outputArray,
		int index) {
	if (!encryptedBlock || !outputArray) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	if (!encryptedBlock->operations->cipherText) {
		return ENCRYPTEDBLOCK_NO_ELEMENT;
	}
	return encryptedBlockConvertMessage(
			arrayComplexGetData(encryptedBlock->operations->cipherText, index,
					outputArray, size));
}
int encryptedBlockGetNameLen(EncryptedBlockPtr encryptedBlock) {
	if (!encryptedBlock || !encryptedBlock->name) {
		return 0;
	}
	return arrayGetLen(encryptedBlock->name);
}
int encryptedBlockGetKeyLen(EncryptedBlockPtr encryptedBlock) {
	if (!encryptedBlock || !encryptedBlock->key) {
		return 0;
	}
	return arrayGetLen(encryptedBlock->key);
}
int encryptedBlockGetAuthKeyLen(EncryptedBlockPtr encryptedBlock) {
	if (!encryptedBlock || !encryptedBlock->authKey) {
		return 0;
	}
	return arrayGetLen(encryptedBlock->authKey);
}
int encryptedBlockGetIvLen(EncryptedBlockPtr encryptedBlock, int index) {
	if (!encryptedBlock || !encryptedBlock->operations->iv) {
		return 0;
	}
	return arrayComplexGetLen(encryptedBlock->operations->iv, index);
}
int encryptedBlockGetIcbLen(EncryptedBlockPtr encryptedBlock, int index) {
	if (!encryptedBlock || !encryptedBlock->operations->icb) {
		return 0;
	}
	return arrayComplexGetLen(encryptedBlock->operations->icb, index);
}
int encryptedBlockGetPlainTextLen(EncryptedBlockPtr encryptedBlock, int index) {
	if (!encryptedBlock || !encryptedBlock->operations->plainText) {
		return 0;
	}
	return arrayComplexGetLen(encryptedBlock->operations->plainText, index);
}
int encryptedBlockGetCipherTextLen(EncryptedBlockPtr encryptedBlock, int index) {
	if (!encryptedBlock || !encryptedBlock->operations->cipherText) {
		return 0;
	}
	return arrayComplexGetLen(encryptedBlock->operations->cipherText, index);
}
int encryptedBlockGetTestCounter(EncryptedBlockPtr encryptedBlock) {
	if (!encryptedBlock) {
		return 0;
	}
	return encryptedBlock->testCounter;
}
int encryptedBlockGetCryptoOffset(EncryptedBlockPtr encryptedBlock, int index) {
	if (!encryptedBlock || !encryptedBlock->operations->cryptoOffset) {
		return 0;
	}
	if(index < 0 || index >= encryptedBlock->operations->operationCounter){
		return 0;
	}
	return encryptedBlock->operations->cryptoOffset[index];
}
int encryptedBlockGetOperationCounter(EncryptedBlockPtr encryptedBlock) {
	if (!encryptedBlock) {
		return 0;
	}
	return encryptedBlock->operations->operationCounter;
}
EncryptedBlockMode encryptedBlockGetMode(EncryptedBlockPtr encryptedBlock) {
	if (!encryptedBlock) {
		return MODE_NULL;
	}
	return encryptedBlock->mode;
}
EncryptedBlockAlgorithm encryptedBlockGetAlgorithm(
		EncryptedBlockPtr encryptedBlock) {

	if (!encryptedBlock) {
		return ALGORITHM_NULL;
	}
	return encryptedBlock->algorithm;
}
EncryptedBlockAuthAlgorithm encryptedBlockGetAuthAlgorithm(
		EncryptedBlockPtr encryptedBlock) {
	if (!encryptedBlock) {
		return AUTH_NULL;
	}
	return encryptedBlock->authAlgorithm;
}
EncryptedBlockDirection encryptedBlockGetDirection(
		EncryptedBlockPtr encryptedBlock) {
	if (!encryptedBlock) {
		return DIRECTION_NULL;
	}
	return encryptedBlock->direction;
}
static EncryptedBlockMessage copyOperations(Operations src, Operations dst) {
	dst->operationCounter = src->operationCounter;
	dst->cryptoOffset = copyIntArray(src->cryptoOffset, src->operationCounter);
	if (src->iv) {
		dst->iv = arrayComplexCopy(src->iv);
		if (!dst->iv) {
			return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
		}
	}
	if (src->icb) {
		dst->icb = arrayComplexCopy(src->icb);
		if (!dst->icb) {
			return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
		}
	}
	if (src->plainText) {
		dst->plainText = arrayComplexCopy(src->plainText);
		if (!dst->plainText) {
			return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
		}
	}
	if (src->cipherText) {
		dst->cipherText = arrayComplexCopy(src->cipherText);
		if (!dst->cipherText) {
			return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
		}
	}
	return ENCRYPTEDBLOCK_SUCCESS;
}
static EncryptedBlockMessage encryptedBlockConvertMessage(ArrayMessage message) {
	switch (message) {
	case ARRAY_SUCCESS:
		return ENCRYPTEDBLOCK_SUCCESS;
		break;
	case ARRAY_NULL_ARGS:
		return ENCRYPTEDBLOCK_NULL_ARGS;
		break;
	case ARRAY_NOT_VALID_ARGS:
		return ENCRYPTEDBLOCK_NOT_VALID_ARGS;
		break;
	case ARRAY_OUT_OF_MEMORY:
		return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
		break;
	case ARRAY_FULL:
		return ENCRYPTEDBLOCK_NOT_ENOUGH_OPERATIONS;
		break;
	case ARRAY_NO_ELEMENT:
		return ENCRYPTEDBLOCK_NO_ELEMENT;
		break;
	case ARRAY_NOT_RIGHT_SIZE:
		return ENCRYPTEDBLOCK_NOT_RIGHT_SIZE;
		break;
	default:
		return ENCRYPTEDBLOCK_NOT_VALID;
		break;
	}
	return ENCRYPTEDBLOCK_NOT_VALID;
}
