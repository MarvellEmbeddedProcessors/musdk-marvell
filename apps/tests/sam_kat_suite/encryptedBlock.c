/******************************************************************************
 *      Copyright (C) 2016 Marvell International Ltd.
 *
 *  If you received this File from Marvell, you may opt to use, redistribute
 *  and/or modify this File under the following licensing terms.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *
 *      * Neither the name of Marvell nor the names of its contributors may be
 *        used to endorse or promote products derived from this software
 *        without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

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
 * algorithm - The algorithm name.
 * mode - The mode name.
 * authAlgorithm - The authentication algorithm name.
 * direction - The direction name.
 * name - The name of the encrypted block.
 * key - The key of the encrypted block.
 * authKey - The authentication key of the encrypted block.
 * operations - the operations of the current encrypted block.
 * testCounter - The number of loop test for each operation.
 */
struct encryptedBlock_t {
	char algorithm[16];
	char mode[16];
	char authAlgorithm[16];
	char direction[16];
	char name[64];
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
	encryptedBlock->key = NULL;
	encryptedBlock->authKey = NULL;
	memset(encryptedBlock->name, 0, sizeof(encryptedBlock->name));
	memset(encryptedBlock->algorithm, 0, sizeof(encryptedBlock->algorithm));
	memset(encryptedBlock->mode, 0, sizeof(encryptedBlock->mode));
	memset(encryptedBlock->authAlgorithm, 0, sizeof(encryptedBlock->authAlgorithm));
	memset(encryptedBlock->direction, 0, sizeof(encryptedBlock->direction));
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

	strcpy((*copyBlock)->algorithm, srcBlock->algorithm);
	strcpy((*copyBlock)->authAlgorithm, srcBlock->authAlgorithm);
	strcpy((*copyBlock)->mode, srcBlock->mode);
	strcpy((*copyBlock)->direction, srcBlock->direction);
	strcpy((*copyBlock)->name, srcBlock->name);

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
	case KEY:
	case AUTH_KEY:
		return encryptedBlockAddArray(type, encryptedBlock, elementArray,
				elementLen);
		break;
	case TESTCOUNTER:
		encryptedBlock->testCounter = elementLen;
		break;
	case NAME:
		if (encryptedBlock->name[0] != '\0')
			return ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST;

		strncpy(encryptedBlock->name, (char *)elementArray, sizeof(encryptedBlock->name));
		break;
	case ALGORITHM:
		if (encryptedBlock->algorithm[0] != '\0')
			return ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST;

		strncpy(encryptedBlock->algorithm, (char *)elementArray, sizeof(encryptedBlock->algorithm));
		break;
	case AUTH_ALGORITHM:
		if (encryptedBlock->authAlgorithm[0] != '\0')
			return ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST;

		strncpy(encryptedBlock->authAlgorithm, (char *)elementArray, sizeof(encryptedBlock->authAlgorithm));
		break;
	case MODE:
		if (encryptedBlock->mode[0] != '\0')
			return ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST;

		strncpy(encryptedBlock->mode, (char *)elementArray, sizeof(encryptedBlock->mode));
		break;
	case DIRECTION:
		if (encryptedBlock->direction[0] != '\0')
			return ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST;

		strncpy(encryptedBlock->direction, (char *)elementArray, sizeof(encryptedBlock->direction));
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
char *encryptedBlockGetName(EncryptedBlockPtr encryptedBlock)
{
	if (!encryptedBlock) {
		return NULL;
	}
	return encryptedBlock->name;
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
char *encryptedBlockGetMode(EncryptedBlockPtr encryptedBlock) {
	if (!encryptedBlock) {
		return NULL;
	}
	return encryptedBlock->mode;
}
char *encryptedBlockGetAlgorithm(
		EncryptedBlockPtr encryptedBlock) {

	if (!encryptedBlock) {
		return NULL;
	}
	return encryptedBlock->algorithm;
}
char *encryptedBlockGetAuthAlgorithm(
		EncryptedBlockPtr encryptedBlock) {
	if (!encryptedBlock) {
		return NULL;
	}
	return encryptedBlock->authAlgorithm;
}
char *encryptedBlockGetDirection(
		EncryptedBlockPtr encryptedBlock) {
	if (!encryptedBlock) {
		return NULL;
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
