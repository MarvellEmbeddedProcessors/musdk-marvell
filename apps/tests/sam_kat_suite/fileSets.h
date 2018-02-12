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
