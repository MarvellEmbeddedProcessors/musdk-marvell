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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "mv_std.h"
#include "lib/lib_misc.h"

#include "encryptedBlock.h"
#include "common.h"
#include "array.h"

///////////////////////////////////////////////////////////////////////////////
/**
 * encryptedBlockReset: resets the given encrypted block.
 *
 * @param encryptedBlock - The encrypted block to be reseted
 */
static void encryptedBlockReset(EncryptedBlockPtr encryptedBlock);

static EncryptedBlockMessage copyOperation(struct operation *src, struct operation *dst);
///////////////////////////////////////////////////////////////////////////////

EncryptedBlockMessage encryptedBlockCreate(
		EncryptedBlockPtr *outputEncryptedBlock)
{
	if (!outputEncryptedBlock) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	*outputEncryptedBlock = malloc(sizeof(*(*outputEncryptedBlock)));
	if (*outputEncryptedBlock == NULL) {
		return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
	}
	encryptedBlockReset(*outputEncryptedBlock);
	return ENCRYPTEDBLOCK_SUCCESS;
}

static void encryptedBlockReset(EncryptedBlockPtr encryptedBlock)
{
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
	encryptedBlock->operationCounter = 0;
	memset(encryptedBlock->operations, 0, sizeof(encryptedBlock->operations));
}

EncryptedBlockMessage encryptedBlockCopy(EncryptedBlockPtr srcBlock,
					 EncryptedBlockPtr *copyBlock)
{
	int i;

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
	for (i = 0; i < srcBlock->operationCounter; i++) {
		if (copyOperation(srcBlock->operations[i], (*copyBlock)->operations[i])
				!= ENCRYPTEDBLOCK_SUCCESS) {
			encryptedBlockDestroy(*copyBlock);
			*copyBlock = NULL;
			return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
		}
	}
	return ENCRYPTEDBLOCK_SUCCESS;
}
void encryptedBlockDestroy(EncryptedBlockPtr encryptedBlock)
{
	int i = 0;

	if (encryptedBlock == NULL)
		return;

	while (encryptedBlock->operations[i]) {
		/* Destroy operation */
		arrayDestroy(encryptedBlock->operations[i]->iv);
		arrayDestroy(encryptedBlock->operations[i]->aad);
		arrayDestroy(encryptedBlock->operations[i]->icb);
		arrayDestroy(encryptedBlock->operations[i]->plainText);
		arrayDestroy(encryptedBlock->operations[i]->cipherText);
		i++;
		free(encryptedBlock->operations[i]);
	}
	arrayDestroy(encryptedBlock->key);
	arrayDestroy(encryptedBlock->authKey);
	free(encryptedBlock);
}

EncryptedBlockMessage encryptedBlockSessionAddElement(EncryptedBlockPtr encryptedBlock,
		EncryptedBlockType type, unsigned char *elementArray, int elementLen)
{
	if (!encryptedBlock || (!elementArray &&
				(type != TESTCOUNTER))) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	switch (type) {
	case KEY:
		if (encryptedBlock->key)
			return ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST;

		encryptedBlock->key = arrayCreate(elementArray, elementLen);
		if (!encryptedBlock->key)
			return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
		break;
	case AUTH_KEY:
		if (encryptedBlock->authKey)
			return ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST;

		encryptedBlock->authKey = arrayCreate(elementArray, elementLen);
		if (!encryptedBlock->authKey)
			return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
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
	default:
		return ENCRYPTEDBLOCK_NOT_VALID_ARGS;
	}
	return ENCRYPTEDBLOCK_SUCCESS;
}

bool encryptedBlockOperationExist(EncryptedBlockPtr encryptedBlock, int index)
{
	if (encryptedBlock && encryptedBlock->operations[index])
		return true;

	return false;
}

EncryptedBlockMessage encryptedBlockOperationCreate(EncryptedBlockPtr encryptedBlock,
						int index)
{
	if (index >= 16)
		return ENCRYPTEDBLOCK_NOT_ENOUGH_OPERATIONS;

	encryptedBlock->operations[index] = malloc(sizeof(struct operation));
	if (!encryptedBlock->operations[index])
		return ENCRYPTEDBLOCK_OUT_OF_MEMORY;

	memset(encryptedBlock->operations[index], 0, sizeof(struct operation));

	return ENCRYPTEDBLOCK_SUCCESS;
}

static ArrayPtr *encryptedBlockOperationArrayPtrGet(EncryptedBlockPtr encryptedBlock,
					     EncryptedBlockType type, int index)
{
	if (!encryptedBlock || !encryptedBlock->operations[index])
		return NULL;

	switch (type) {
	case IV:
		return &encryptedBlock->operations[index]->iv;
	case ICB:
		return &encryptedBlock->operations[index]->icb;
	case AAD:
		return &encryptedBlock->operations[index]->aad;
	case PLAINTEXT:
		return &encryptedBlock->operations[index]->plainText;
	case CIPHERTEXT:
		return &encryptedBlock->operations[index]->cipherText;
	default:
		break;
	}
	return NULL;
}

EncryptedBlockMessage encryptedBlockOperationAddElement(EncryptedBlockPtr encryptedBlock,
		EncryptedBlockType type, int index, unsigned char *elementArray, int elementLen)
{
	ArrayPtr *element_ptr;

	if (!encryptedBlock || !encryptedBlock->operations[index])
		return ENCRYPTEDBLOCK_NULL_ARGS;

	if (!elementArray && ((type != CRYPTO_OFFSET) &&
			      (type != AUTH_OFFSET))) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}

	switch (type) {
	case IV:
	case ICB:
	case AAD:
	case PLAINTEXT:
	case CIPHERTEXT:
		element_ptr = encryptedBlockOperationArrayPtrGet(encryptedBlock, type, index);
		if (!element_ptr) {
			return ENCRYPTEDBLOCK_NULL_ARGS;
		}

		if (*element_ptr != NULL) {
			printf("Element of operation type = %d already exist for index = %d\n",
				type, index);
			return ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST;
		}

		*element_ptr = arrayCreate(elementArray, elementLen);
		if (*element_ptr == NULL) {
			return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
		}
		break;
	case CRYPTO_OFFSET:
		encryptedBlock->operations[index]->cryptoOffset = elementLen;
		break;
	case AUTH_OFFSET:
		encryptedBlock->operations[index]->authOffset = elementLen;
		break;
	case TEXT_LEN:
		encryptedBlock->operations[index]->textLen = elementLen;
		break;
	default:
		printf("Unexpected operation type = %d\n", type);
		return ENCRYPTEDBLOCK_NOT_VALID_ARGS;
	}
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
		int size, unsigned char *outputArray)
{
	if (!encryptedBlock || !outputArray) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	if (!encryptedBlock->key) {
		return ENCRYPTEDBLOCK_NO_ELEMENT;
	}
	arrayGetData(encryptedBlock->key, outputArray, size);

	return ENCRYPTEDBLOCK_SUCCESS;
}

EncryptedBlockMessage encryptedBlockGetAuthKey(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char *outputArray)
{
	if (!encryptedBlock || !outputArray) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	if (!encryptedBlock->authKey) {
		return ENCRYPTEDBLOCK_NO_ELEMENT;
	}
	arrayGetData(encryptedBlock->authKey, outputArray, size);

	return ENCRYPTEDBLOCK_SUCCESS;
}

EncryptedBlockMessage encryptedBlockGetIv(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char *outputArray, int index)
{
	if (!encryptedBlock || !encryptedBlock->operations[index] || !outputArray) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	if (!encryptedBlock->operations[index]->iv) {
		return ENCRYPTEDBLOCK_NO_ELEMENT;
	}
	arrayGetData(encryptedBlock->operations[index]->iv, outputArray, size);

	return ENCRYPTEDBLOCK_SUCCESS;
}

EncryptedBlockMessage encryptedBlockGetIcb(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char *outputArray, int index)
{
	if (!encryptedBlock || !encryptedBlock->operations[index] || !outputArray) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	if (!encryptedBlock->operations[index]->icb) {
		return ENCRYPTEDBLOCK_NO_ELEMENT;
	}
	arrayGetData(encryptedBlock->operations[index]->icb, outputArray, size);

	return ENCRYPTEDBLOCK_SUCCESS;
}

EncryptedBlockMessage encryptedBlockGetAad(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char *outputArray, int index)
{
	if (!encryptedBlock || !encryptedBlock->operations[index] || !outputArray) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	if (!encryptedBlock->operations[index]->aad) {
		return ENCRYPTEDBLOCK_NO_ELEMENT;
	}
	arrayGetData(encryptedBlock->operations[index]->aad, outputArray, size);

	return ENCRYPTEDBLOCK_SUCCESS;
}

EncryptedBlockMessage encryptedBlockGetPlainText(
		EncryptedBlockPtr encryptedBlock, int size, unsigned char* outputArray,
		int index)
{
	int copylen;

	if (!encryptedBlock || !encryptedBlock->operations[index] || !outputArray) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	if (!encryptedBlock->operations[index]->plainText) {
		return ENCRYPTEDBLOCK_NO_ELEMENT;
	}
	while (size) {
		copylen = arrayGetData(encryptedBlock->operations[index]->plainText, outputArray, size);
		outputArray += copylen;
		size -= copylen;
	}
	return ENCRYPTEDBLOCK_SUCCESS;
}

EncryptedBlockMessage encryptedBlockGetCipherText(
		EncryptedBlockPtr encryptedBlock, int size, unsigned char* outputArray,
		int index)
{
	int copylen;

	if (!encryptedBlock || !encryptedBlock->operations[index] || !outputArray) {
		return ENCRYPTEDBLOCK_NULL_ARGS;
	}
	if (!encryptedBlock->operations[index]->cipherText) {
		/* TBD: Generate random data */
		return ENCRYPTEDBLOCK_NO_ELEMENT;
	}
	while (size) {
		copylen = arrayGetData(encryptedBlock->operations[index]->cipherText, outputArray, size);
		outputArray += copylen;
		size -= copylen;
	}
	return ENCRYPTEDBLOCK_SUCCESS;
}

int encryptedBlockGetKeyLen(EncryptedBlockPtr encryptedBlock)
{
	if (!encryptedBlock || !encryptedBlock->key) {
		return 0;
	}
	return arrayGetLen(encryptedBlock->key);
}

int encryptedBlockGetAuthKeyLen(EncryptedBlockPtr encryptedBlock)
{
	if (!encryptedBlock || !encryptedBlock->authKey) {
		return 0;
	}
	return arrayGetLen(encryptedBlock->authKey);
}

int encryptedBlockGetIvLen(EncryptedBlockPtr encryptedBlock, int index)
{
	if (!encryptedBlock || !encryptedBlock->operations[index] ||
	    !encryptedBlock->operations[index]->iv) {
		return 0;
	}
	return arrayGetLen(encryptedBlock->operations[index]->iv);
}

int encryptedBlockGetAadLen(EncryptedBlockPtr encryptedBlock, int index)
{
	if (!encryptedBlock || !encryptedBlock->operations[index] ||
	    !encryptedBlock->operations[index]->aad) {
		return 0;
	}
	return arrayGetLen(encryptedBlock->operations[index]->aad);
}

int encryptedBlockGetIcbLen(EncryptedBlockPtr encryptedBlock, int index)
{
	if (!encryptedBlock || !encryptedBlock->operations[index] ||
	    !encryptedBlock->operations[index]->icb) {
		return 0;
	}
	return arrayGetLen(encryptedBlock->operations[index]->icb);
}

int encryptedBlockGetPlainTextLen(EncryptedBlockPtr encryptedBlock, int index)
{
	if (!encryptedBlock || !encryptedBlock->operations[index])
		return 0;

	if (encryptedBlock->operations[index]->textLen)
		return encryptedBlock->operations[index]->textLen;

	if (encryptedBlock->operations[index]->plainText)
		return arrayGetLen(encryptedBlock->operations[index]->plainText);

	return 0;
}

int encryptedBlockGetCipherTextLen(EncryptedBlockPtr encryptedBlock, int index)
{
	if (!encryptedBlock || !encryptedBlock->operations[index])
		return 0;

	if (encryptedBlock->operations[index]->textLen)
		return encryptedBlock->operations[index]->textLen;

	if (encryptedBlock->operations[index]->cipherText)
		return arrayGetLen(encryptedBlock->operations[index]->cipherText);

	return 0;
}

int encryptedBlockGetTestCounter(EncryptedBlockPtr encryptedBlock)
{
	if (!encryptedBlock) {
		return 0;
	}
	return encryptedBlock->testCounter;
}

int encryptedBlockGetCryptoOffset(EncryptedBlockPtr encryptedBlock, int index)
{
	if (!encryptedBlock || !encryptedBlock->operations[index]) {
		return 0;
	}
	return encryptedBlock->operations[index]->cryptoOffset;
}

int encryptedBlockGetAuthOffset(EncryptedBlockPtr encryptedBlock, int index)
{
	if (!encryptedBlock || !encryptedBlock->operations[index]) {
		return 0;
	}
	return encryptedBlock->operations[index]->authOffset;
}

char *encryptedBlockGetMode(EncryptedBlockPtr encryptedBlock)
{
	if (!encryptedBlock) {
		return NULL;
	}
	return encryptedBlock->mode;
}

char *encryptedBlockGetAlgorithm(EncryptedBlockPtr encryptedBlock)
{
	if (!encryptedBlock) {
		return NULL;
	}
	return encryptedBlock->algorithm;

}
char *encryptedBlockGetAuthAlgorithm(EncryptedBlockPtr encryptedBlock)
{
	if (!encryptedBlock) {
		return NULL;
	}
	return encryptedBlock->authAlgorithm;
}

char *encryptedBlockGetDirection(EncryptedBlockPtr encryptedBlock)
{
	if (!encryptedBlock) {
		return NULL;
	}
	return encryptedBlock->direction;
}

static EncryptedBlockMessage copyOperation(struct operation *src, struct operation *dst)
{
	dst->cryptoOffset = src->cryptoOffset;
	if (src->iv) {
		dst->iv = arrayCopy(src->iv);
		if (!dst->iv) {
			return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
		}
	}
	if (src->icb) {
		dst->icb = arrayCopy(src->icb);
		if (!dst->icb) {
			return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
		}
	}
	if (src->plainText) {
		dst->plainText = arrayCopy(src->plainText);
		if (!dst->plainText) {
			return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
		}
	}
	if (src->cipherText) {
		dst->cipherText = arrayCopy(src->cipherText);
		if (!dst->cipherText) {
			return ENCRYPTEDBLOCK_OUT_OF_MEMORY;
		}
	}
	return ENCRYPTEDBLOCK_SUCCESS;
}
