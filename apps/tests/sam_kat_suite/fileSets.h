/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef FILESETS_H_
#define FILESETS_H_

#include "encryptedBlock.h"

/** Type used for returning message from fileSets functions */
typedef enum {
	FILE_NULL_ARGS,
	FILE_OPEN_PROBLEM,
	FILE_SUCCESS,
	FILE_NOT_VALID,
	FILE_OUT_OF_MEMORY,
	FILE_TOO_BIG_PHRASE,
	FILE_ELEMENT_ALREADY_EXIST,
	FILE_TYPE_NOT_VALID
} FileMessage;

/**
 * fileSetsReadBlocksFromFile: Reads the encrypted blocks from a specified file.
 *
 * @param fileName - The file name to read from
 * @param encryptedBlocks - The list to add the encrypted blocks to.
 * @param maxBlocksNum - max number of blocks
 * @return
 * 	FILE_NULL_ARGS - If at least one of the parameters is NULL
 * 	FILE_OPEN_PROBLEM - If the function can'ot open the specified file
 * 	FILE_TOO_BIG_PHRASE - If there is a phrase to get that is bigger than
 * 		CHUNK_SIZE (who's defined)
 * 	FILE_OUT_OF_MEMORY- If an allocation is failed
 * 	FILE_ELEMENT_ALREADY_EXIST - If an element is already exist in the current
 * 		encrypted block
 * 	FILE_NOT_VALID - If other error occurred
 * 	FILE_SUCCESS - If the function succeeded to read the blocks from the file
 */
FileMessage fileSetsReadBlocksFromFile(char *fileName, EncryptedBlockPtr *encryptedBlocks, int maxBlocksNum);

#endif /* FILESETS_H_ */
