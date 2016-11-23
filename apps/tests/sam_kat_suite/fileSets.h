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

#ifndef FILESETS_H_
#define FILESETS_H_

#include "generic_list.h"

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

/** Defines the type that declares on a new encrypted block */
#define NEW_BLOCK_TYPE ALGORITHM

/**
 * fileSetsReadBlocksFromFile: Reads the encrypted blocks from a specified file.
 *
 * @param fileName - The file name to read from
 * @param encryptedBlocks - The list to add the encrypted blocks to.
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
FileMessage fileSetsReadBlocksFromFile(char* fileName, generic_list encryptedBlocks);

/* cast functions for generic list */
generic_list_element fileSetsEncryptedBlockCopyForList(generic_list_element block);
void fileSetsEncryptedBlockDestroyForList(generic_list_element encryptedBlock);

#endif /* FILESETS_H_ */
