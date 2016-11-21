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

#ifndef ENCRYPTEDBLOCK_H_
#define ENCRYPTEDBLOCK_H_

/** Type for defining the encrypted block */
typedef struct encryptedBlock_t* EncryptedBlockPtr;

/** Type for defining the encrypted block type of element */
typedef enum {
	INVALID,
	NAME,
	KEY,
	AUTH_KEY,
	IV,
	ICB,
	PLAINTEXT,
	CIPHERTEXT,
	TESTCOUNTER,
	OPERATIONCOUNTER,
	ALGORITHM,
	MODE,
	AUTH_ALGORITHM,
	DIRECTION,
	CRYPTO_OFFSET
} EncryptedBlockType;

/** Type for defining the encrypted block algorithm */
typedef enum {
	ALGORITHM_NULL,
	ALGORITHM_DES,
	ALGORITHM_3DES,
	ALGORITHM_AES,
	ALGORITHM_INVALID
} EncryptedBlockAlgorithm;

/** Type for defining the encrypted block mode */
typedef enum {
	MODE_NULL, MODE_ECB, MODE_CBC, MODE_CTR, MODE_GCM, MODE_GMAC, MODE_INVALID
} EncryptedBlockMode;

/** Type for defining the encrypted block auth algorithm */
typedef enum {
	AUTH_NULL,
	AUTH_MD5,
	AUTH_SHA1,
	AUTH_SHA224,
	AUTH_SHA256,
	AUTH_SHA384,
	AUTH_SHA512,
	AUTH_INVALID
} EncryptedBlockAuthAlgorithm;

/** Type for defining the encrypted block direction */
typedef enum {
	DIRECTION_NULL, ENCRYPTION, DECRYPTION, DIRECTION_INVALID
} EncryptedBlockDirection;

/** Type used for returning message from encrypted block functions */
typedef enum {
	ENCRYPTEDBLOCK_NULL_ARGS,
	ENCRYPTEDBLOCK_SUCCESS,
	ENCRYPTEDBLOCK_NOT_VALID,
	ENCRYPTEDBLOCK_NOT_VALID_TYPE,
	ENCRYPTEDBLOCK_NOT_VALID_ARGS,
	ENCRYPTEDBLOCK_NO_ELEMENT,
	ENCRYPTEDBLOCK_NOT_RIGHT_SIZE,
	ENCRYPTEDBLOCK_OUT_OF_MEMORY,
	ENCRYPTEDBLOCK_TOO_BIG_STRING,
	ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST,
	ENCRYPTEDBLOCK_NEGATIVE_COUNTER,
	ENCRYPTEDBLOCK_NOT_ENOUGH_OPERATIONS
} EncryptedBlockMessage;

/**
 * encryptedBlockCreate: Creates a new encrypted block.
 *
 * @param encryptedBlock - The encrypted block to be created.
 * @return
 *	ENCRYPTEDBLOCK_NULL_ARGS - If at least one of the parameters is NULL
 * 	ENCRYPTEDBLOCK_OUT_OF_MEMORY - If an allocation failed
 * 	ENCRYPTEDBLOCK_SUCCESS - If the function succeeded to create a new block
 */
EncryptedBlockMessage encryptedBlockCreate(EncryptedBlockPtr* encryptedBlock);
/**
 * encryptedBlockCopy: Creates a copy of the encrypted block.
 *
 * @param srcBlock - The encrypted block to be copied.
 * @param copyBlock - pointer to the copied encrypted block.
 * 	This is an output parameter.
 * @return
 * 	ENCRYPTEDBLOCK_NULL_ARGS - If at least one of the parameters is NULL
 * 	ENCRYPTEDBLOCK_OUT_OF_MEMORY - If an allocation failed
 * 	ENCRYPTEDBLOCK_SUCCESS - If the function succeeded to copy the block
 */
EncryptedBlockMessage encryptedBlockCopy(EncryptedBlockPtr srcBlock,
		EncryptedBlockPtr* copyBlock);

/**
 * encryptedBlockAddElement: Adds an element to the encryptedBlock.
 *
 * @param encryptedBlock - The encrypted block to add the element to.
 * @param encryptedBlockType - The type of the element.
 * @param element - The element array to be added.
 * @param elementLen - The lengh of the element array.
 * @return
 * 	ENCRYPTEDBLOCK_NULL_ARGS - If at least one of the parameters is NULL
 * 	ENCRYPTEDBLOCK_OUT_OF_MEMORY - If an allocation failed
 * 	ENCRYPTEDBLOCK_NOT_VALID_TYPE - If the type is not valid encrypted block type
 *	ENCRYPTEDBLOCK_NOT_VALID_ARGS - If at least one of the parameters is not valid
 * 	ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST: If the specified type of element
 * 		already inserted to the block
 *	ENCRYPTEDBLOCK_NOT_ENOUGH_OPERATIONS - If the array is full
 * 	ENCRYPTEDBLOCK_SUCCESS - If the function succeeded to add the element
 */
EncryptedBlockMessage encryptedBlockAddElement(EncryptedBlockPtr encryptedBlock,
		EncryptedBlockType encryptedBlockType, unsigned char* elementIntArray,
		int elementLen);

/**
 * encryptedBlockDestroy: Destroys the given encrypted block.
 *
 * @param encryptedBlock - The encrypted block to be destroyed.
 */
void encryptedBlockDestroy(EncryptedBlockPtr encryptedBlock);
/**
 * encryptedBlockSetTestCounter: Sets the encrypted test counter.
 *
 * @param encryptedBlock - The encrypted block to be set.
 * @param testCounter - The element value to insert.
 * @return
 *	ENCRYPTEDBLOCK_NULL_ARGS - If encryptedBlock is NULL
 * 	ENCRYPTEDBLOCK_NEGATIVE_COUNTER - If the value is negative
 * 	ENCRYPTEDBLOCK_SUCCESS - If the function succeeded
 */
EncryptedBlockMessage encryptedBlockSetTestCounter(
		EncryptedBlockPtr encryptedBlock, int testCounter);
/**
 * encryptedBlockSetCryptoOffset: Sets the next in line encrypted block crypto offset.
 *
 * @param encryptedBlock - The encrypted block to be set.
 * @param crypto offset - The element value to insert.
 * @return
 *	ENCRYPTEDBLOCK_NULL_ARGS - If encryptedBlock is NULL
 * 	ENCRYPTEDBLOCK_NEGATIVE_COUNTER - If the value is negative
 * 	ENCRYPTEDBLOCK_NOT_ENOUGH_OPERATIONS - If the array is full
 * 	ENCRYPTEDBLOCK_SUCCESS - If the function succeeded
 */
EncryptedBlockMessage encryptedBlockSetCryptoOffset(
		EncryptedBlockPtr encryptedBlock, int off);

/**
 * encryptedBlockGet(ELEMENT): copy the encrypted block element to the
 * 		output array.
 *
 * @param encryptedBlock - The encrypted block to copy the element from.
 * @param size - The size of the output array.
 * @param outputArray - The output array to copy the element to.
 * only for cipher text and plain text:
 * (@param index - The index of the array to be copied)
 * @return
 * 	ENCRYPTEDBLOCK_NULL_ARGS - If at least one of the parameters is NULL
 * 	ENCRYPTEDBLOCK_NO_ELEMENT - If the element doesn't exist in the block
 * 	ENCRYPTEDBLOCK_NOT_RIGHT_SIZE - If size of the output array doesn't match
 * 		the element length
 * 	ENCRYPTEDBLOCK_SUCCESS - If the function succeeded to copy the element
 */
EncryptedBlockMessage encryptedBlockGetName(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char* outputArray);
EncryptedBlockMessage encryptedBlockGetKey(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char* outputArray);
EncryptedBlockMessage encryptedBlockGetAuthKey(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char* outputArray);
EncryptedBlockMessage encryptedBlockGetIv(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char* outputArray, int index);
EncryptedBlockMessage encryptedBlockGetIcb(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char* outputArray, int index);
EncryptedBlockMessage encryptedBlockGetPlainText(
		EncryptedBlockPtr encryptedBlock, int size, unsigned char* outputArray,
		int index);
EncryptedBlockMessage encryptedBlockGetCipherText(
		EncryptedBlockPtr encryptedBlock, int size, unsigned char* outputArray,
		int index);

/**
 * encryptedBlockGet(ELEMENT)Len: Gets the number of bytes of the encrypted block element.
 *
 * @param encryptedBlock - The encrypted block to get the length of the element
 * 	 from.
 * (@param index - The index of the wanted array)
 * @return
 * 	the element length
 */
int encryptedBlockGetNameLen(EncryptedBlockPtr encryptedBlock);
int encryptedBlockGetKeyLen(EncryptedBlockPtr encryptedBlock);
int encryptedBlockGetAuthKeyLen(EncryptedBlockPtr encryptedBlock);
int encryptedBlockGetIvLen(EncryptedBlockPtr encryptedBlock, int index);
int encryptedBlockGetIcbLen(EncryptedBlockPtr encryptedBlock, int index);
int encryptedBlockGetPlainTextLen(EncryptedBlockPtr encryptedBlock, int index);
int encryptedBlockGetCipherTextLen(EncryptedBlockPtr encryptedBlock, int index);
/**
 * encryptedBlockGet(ELEMENT): Gets the numeric value of the encrypted block element.
 *
 * @param encryptedBlock - The encrypted block to get the length of the element
 * 	 from.
 * (@param index - The index of the wanted array)
 * @return
 * 	the element value
 */
int encryptedBlockGetTestCounter(EncryptedBlockPtr encryptedBlock);
int encryptedBlockGetOperationCounter(EncryptedBlockPtr encryptedBlock);
int encryptedBlockGetCryptoOffset(EncryptedBlockPtr encryptedBlock, int index);
/**
 * encryptedBlockGet(ELEMENT): Gets the encrypted block element.
 *
 * @param encryptedBlock - The encrypted block to get the element from.
 * @return
 * 	the element type
 */
EncryptedBlockMode encryptedBlockGetMode(EncryptedBlockPtr encryptedBlock);
EncryptedBlockAlgorithm encryptedBlockGetAlgorithm(
		EncryptedBlockPtr encryptedBlock);
EncryptedBlockAuthAlgorithm encryptedBlockGetAuthAlgorithm(
		EncryptedBlockPtr encryptedBlock);
EncryptedBlockDirection encryptedBlockGetDirection(
		EncryptedBlockPtr encryptedBlock);

#endif /* ENCRYPTEDBLOCK_H_ */
