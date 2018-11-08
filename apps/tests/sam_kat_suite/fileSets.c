/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#include "fileSets.h"
#include "encryptedBlock.h"
#include "common.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

///////////////////////////////////////////////////////////////////////////////
/**
 * fileSetsGetArgs: gets a new type and its relevant data from the file.
 *
 * @param fstr - The pointer to the file to take the arguments from.
 * @param line - The line to get the arguments from.
 * @param type - The output element type.
 * @param outputDataArray - The element array.
 * @param outputDataNumber - The element array length or the counter value.
 * 	 @return
 * 	FILE_NULL_ARGS - If at least one of the parameters is NULL
 * 	FILE_TOO_BIG_STRING - If the phrase to get is bigger than
 * 		CHUNK_SIZE (who's defined)
 * 	FILE_NOT_VALID - If other error occurred
 * 	FILE_SUCCESS - If the function succeed to get the arguments
 */
static FileMessage fileSetsGetArgs(FILE *fstr, char *line,
		EncryptedBlockType *type, unsigned char *outputDataArray,
		int *outputDataNumber);
/**
 * fileSetsGetCounter: gets a new counter from the given line.
 *
 * @param line - The line to get the counter from.
 * @param outputCounter - The counter value to assign to.
 * 	 @return
 * 	FILE_NOT_VALID - If an error occurred while trying to get the counter
 * 			from the line
 * 	FILE_SUCCESS - If the function succeed to get the counter
 */
static FileMessage fileSetsGetCounter(char *line, int *outputCounter);
/**
 * fileSetsGetString: gets a string from the given line.
 *
 * @param line - The line to get the string from.
 * @param outputDataArray - The array to assign the string to.
 * @param outputDataLen - The length of the string.
 * 	 @return
 * 	FILE_NOT_VALID - If an error occurred while trying to get the string
 * 			from the line
 * 	FILE_SUCCESS - If the function succeed to gets the counter
 */
static FileMessage fileSetsGetString(char *line, unsigned char *outputDataArray,
		int *outputDataLen);
/**
 * fileSetsGetData: gets a new data from the file.
 *
 * @param fstr - The pointer to the file to take the data from
 * @param line - The first line of the data
 * @param dataOut - The element array
 * @param dataOutLen - The element array length
 * 	 @return
 * 	FILE_NULL_ARGS - If at least one of the parameters is NULL
 * 	FILE_NOT_VALID - If the data is not a valid hex phrase
 * 	FILE_TOO_BIG_STRING - If the data to get is bigger than
 * 		CHUNK_SIZE (who's defined)
 * 	FILE_SUCCESS - If the function succeed to get the data
 */
static FileMessage fileSetsGetData(FILE *fstr, char *line,
		unsigned char *outputDataArray, int *outputDataLen);
/**
 * fileSetsGetTypeFromString: Gets the encrypted block type.
 *
 * @param str - The string to get the type from.
 * @return
 *  NAME - If the string is "Name"
 *  KEY - If the string is "Key"
 *  AUTH_KEY - If the string is "Authkey"
 *  IV - If the string is "IV"
 *  ICB - If the string is "ICB"
 *  PLAINTEXT - If the string is "PLAINTEXT" or "PT"
 *  CIPHERTEXT - If the string is "Ciphertext" or "CT"
 *  TESTCOUNTER - If the string is "Testcounter"
 *  MODE - If the string is "Mode"
 *  ALGORITHM - If the string is "Algorithm"
 *  AUTH_ALGORITHM - If the string is "Authalgorithm"
 *  DIRECTION - If the string is "Direction"
 *  CRYPTO_OFFSET - If the string is "Cryptoffset"
 *  INVALID - If other string was sent
 */
static EncryptedBlockType fileSetsGetTypeFromString(char *str);

/**
 * fileSetsCnvertMessage: Converts encrypted block message to file sets message.
 *
 * @param message - The encrypted block message to be converted
 * 	 @return
 * 		the relevant file sets message
 */
static FileMessage fileSetsConvertMessage(EncryptedBlockMessage message);
///////////////////////////////////////////////////////////////////////////////

FileMessage fileSetsReadBlocksFromFile(char *fileName, EncryptedBlockPtr * encryptedBlocks, int maxBlocksNum)
{
	EncryptedBlockMessage encryptedBlockMessage;
	EncryptedBlockPtr currentBlock;
	FILE *fstr;
	char line[CHUNK_SIZE] = { 0 };
	int block_index = 0;

	if (!fileName || !encryptedBlocks) {
		return FILE_NULL_ARGS;
	}

	fstr = fopen(fileName, "r");
	if (fstr == NULL) {
		return FILE_OPEN_PROBLEM;
	}

	while ((fgets(line, CHUNK_SIZE, fstr) != NULL) && (block_index < maxBlocksNum)) {
		EncryptedBlockType type = INVALID;
		unsigned char dataArray[CHUNK_SIZE] = { 0 };
		int dataNumber = 0;
		FileMessage message;
		int operation_index = 0;

		if (isCommentOrEmptyLine(line)) {
			cleanStr(line);
			continue;
		}

		message = fileSetsGetArgs(fstr, line, &type, dataArray, &dataNumber);
		if (message != FILE_SUCCESS) {
			printf("fileSetsGetArgs FAILED - rc = %d, type = %d\n", message, type);
			fclose(fstr);
			return message;
		}

		if (type == NEW_BLOCK_TYPE) {
			encryptedBlockMessage = encryptedBlockCreate(&currentBlock);
			if (encryptedBlockMessage == ENCRYPTEDBLOCK_OUT_OF_MEMORY) {
				fclose(fstr);
				return FILE_OUT_OF_MEMORY;
			}
			encryptedBlocks[block_index++] = currentBlock;
			operation_index = 0;
		}

		if (isEncryptedBlockTypeSession(type)) {
			message = fileSetsConvertMessage(
				encryptedBlockSessionAddElement(currentBlock, type, dataArray,
						dataNumber));
		} else if (isEncryptedBlockTypeOperation(type)) {
			if (!encryptedBlockOperationExist(currentBlock, operation_index)) {
				encryptedBlockMessage = encryptedBlockOperationCreate(currentBlock,
										      operation_index);
				if (encryptedBlockMessage != ENCRYPTEDBLOCK_SUCCESS) {
					fclose(fstr);
					return FILE_OUT_OF_MEMORY;
				}
			}
			message = fileSetsConvertMessage(
				encryptedBlockOperationAddElement(currentBlock, type, operation_index++,
								dataArray, dataNumber));
		} else {
			printf("Unexpected type = %d\n", type);
			message = FILE_NULL_ARGS;
		}
		if (message != FILE_SUCCESS) {
			if (type == NEW_BLOCK_TYPE) {
				encryptedBlockDestroy(currentBlock);
			}
			fclose(fstr);
			return message;
		}
		cleanStr(line);
	}
	fclose(fstr);
	return FILE_SUCCESS;
}
static FileMessage fileSetsGetArgs(FILE *fstr, char *line,
		EncryptedBlockType *type, unsigned char *outputDataArray,
		int *outputDataNumber)
{
	if (!line || !type || !outputDataArray) {
		return FILE_NULL_ARGS;
	}
	char firstArg[CHUNK_SIZE] = { 0 }, secondArg;
	int scanned = sscanf(line, " %s %c", firstArg, &secondArg);
	if (scanned != 2) {
		printf("Wrong scanned %d != 2: line - %s\n", scanned, line);
		return FILE_NOT_VALID;
	}
	*type = fileSetsGetTypeFromString(firstArg);
	if (*type == INVALID || secondArg != ':') {
		printf("Unknown parameter: %s\n", firstArg);
		return FILE_NOT_VALID;
	}
	removeBeforeColon(line);
	switch (*type) {
	case TESTCOUNTER:
	case CRYPTO_OFFSET:
	case AUTH_OFFSET:
	case TEXT_LEN:
		return fileSetsGetCounter(line, outputDataNumber);
		break;
	case NAME:
	case ALGORITHM:
	case AUTH_ALGORITHM:
	case MODE:
	case DIRECTION:
		return fileSetsGetString(line, outputDataArray, outputDataNumber);
		break;
	default:
		return fileSetsGetData(fstr, line, outputDataArray, outputDataNumber);
		break;
	}
}
static FileMessage fileSetsGetCounter(char *line, int *outputCounter)
{
	if (sscanf(line, " %d", outputCounter) != 1) {
		return FILE_NOT_VALID;
	}
	return FILE_SUCCESS;
}
static FileMessage fileSetsGetString(char *line, unsigned char *outputDataArray,
					int *outputDataLen) {
	if (sscanf(line, " %s", outputDataArray) != 1) {
		return FILE_NOT_VALID;
	}
	if (*outputDataArray == '"') {
		if (!getPhraseInQuotationMarks((char*) outputDataArray, line)) {
			return FILE_NOT_VALID;
		}
	}
	*outputDataLen = strlen((char*) outputDataArray) + 1;
	return FILE_SUCCESS;
}

static FileMessage fileSetsGetData(FILE *fstr, char *line,
		unsigned char *outputDataArray, int *outputDataLen)
{
	char strOut[CHUNK_SIZE] = { 0 };
	bool continuesData;
	int scanned, strLen = 0;
	do {
		continuesData = isContinuesData(line);
		char str[CHUNK_SIZE] = { 0 };
		scanned = sscanf(line, "%s", str);
		if (scanned != 1) {
			printf("%s: sscanf Failed - scanned = %d\n", __func__, scanned);
			return FILE_NOT_VALID;
		}
		if (str[0] != '"') {
			removeHexHead(str);
			if (!isValidHexPhrase(str)) {
				return FILE_NOT_VALID;
			}
			if ((strLen + strlen(str)) >= CHUNK_SIZE) {
				return FILE_TOO_BIG_PHRASE;
			}
			strcpy(&strOut[strLen], str);
		} else {
			if (!getPhraseInQuotationMarks(str, line)) {
				return FILE_NOT_VALID;
			}
			if ((strLen + strlen(str)) >= CHUNK_SIZE) {
				return FILE_TOO_BIG_PHRASE;
			}
			strAsci2HexAsci(str, &strOut[strLen]);
			strLen += strlen(str);
		}
		strLen += strlen(str);
		if (continuesData) {
			cleanStr(line);
			if (fgets(line, CHUNK_SIZE, fstr) == NULL) {
				printf("%s: Can't read continues Data\n", __func__);
				return FILE_NOT_VALID;
			}
		}
	} while (continuesData);
	*outputDataLen = strLen / 2;
	hexString2Int(strOut, outputDataArray);
	return FILE_SUCCESS;
}
static EncryptedBlockType fileSetsGetTypeFromString(char* str) {
	if (!str) {
		return INVALID;
	}
	if (strcmp(str, "Name") == 0)
		return NAME;

	if (strcmp(str, "Key") == 0)
		return KEY;

	if (strcmp(str, "Authkey") == 0)
		return AUTH_KEY;

	if (strcmp(str, "IV") == 0)
		return IV;

	if (strcmp(str, "ICB") == 0)
		return ICB;

	if (strcmp(str, "AAD") == 0)
		return AAD;

	if (strcmp(str, "Plaintext") == 0 || strcmp(str, "PT") == 0)
		return PLAINTEXT;

	if (strcmp(str, "Ciphertext") == 0 || strcmp(str, "CT") == 0)
		return CIPHERTEXT;

	if (strcmp(str, "Testcounter") == 0)
		return TESTCOUNTER;

	if (strcmp(str, "Mode") == 0)
		return MODE;

	if (strcmp(str, "Algorithm") == 0)
		return ALGORITHM;

	if (strcmp(str, "Authalgorithm") == 0)
		return AUTH_ALGORITHM;

	if (strcmp(str, "Cryptoffset") == 0)
		return CRYPTO_OFFSET;

	if (strcmp(str, "Authoffset") == 0)
		return AUTH_OFFSET;

	if (strcmp(str, "Direction") == 0)
		return DIRECTION;

	if (strcmp(str, "Textlen") == 0)
		return TEXT_LEN;

	return INVALID;
}

static FileMessage fileSetsConvertMessage(EncryptedBlockMessage message)
{
	switch (message) {
	case ENCRYPTEDBLOCK_SUCCESS:
		return FILE_SUCCESS;
		break;
	case ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST:
		return FILE_ELEMENT_ALREADY_EXIST;
		break;
	case ENCRYPTEDBLOCK_OUT_OF_MEMORY:
		return FILE_OUT_OF_MEMORY;
		break;
	case ENCRYPTEDBLOCK_NOT_VALID_ARGS:
	case ENCRYPTEDBLOCK_NOT_VALID_TYPE:
	case ENCRYPTEDBLOCK_NOT_VALID:
		return FILE_NOT_VALID;
		break;
	case ENCRYPTEDBLOCK_NULL_ARGS:
		return FILE_NULL_ARGS;
		break;
	default:
		return FILE_NOT_VALID;
		break;
	}
	return FILE_NOT_VALID;
}
