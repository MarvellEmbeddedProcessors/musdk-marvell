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

#ifndef COMMON_H_
#define COMMON_H_

#include <stdbool.h>

/* support up to 12Kb buffer */
#define CHUNK_SIZE (1024*12*2)

/* checks if the given string is a valid hex phrase */
bool isValidHexPhrase(char* str);

/* checks if the given char is between 'A' - 'F' */
bool isBigHexLetter(char c);

/* checks if the given char is between 'a' - 'f' */
bool isLittleHexLetter(char c);

/* checks if the given char is between '0' - '9' */
bool isHexNumber(char c);

/* checks if there is an ) in the given string */
bool isEndOfBracketsExist(char* str);

/* checks if there is an \ in the given line */
bool isContinuesData(char* line);

/* checks if the given line is a comment line or empty line*/
bool isCommentOrEmptyLine(char* line);

/* copies to strOut the phrase in quotation marks in line */
bool getPhraseInQuotationMarks(char* strOut, char* line);

/* returns the hex char of the given value */
char getHexChar(int decvalue);

/* returns the dec value of the given hex char */
int hex2dec(char c);

/* returns pointer to allocated copy of dst */
int* copyIntArray(int* src, int len);

/* converts an hex string to is hex value */
void strAsci2HexAsci(char* str, char* strHex);

/* converts an hex string to is dec value */
void hexString2Int(char* str, unsigned char* outResult) ;

/* converts a string to is dec value (ascii value) */
void strArr2IntArr(char* str, int* array);

/* removes 0x from the start of the given string */
void removeHexHead(char* str);

/* removes everything before ':' from the given line */
void removeBeforeColon(char* line);

/* reset the given string */
void cleanStr(char *var);

#endif /* COMMON_H_ */
