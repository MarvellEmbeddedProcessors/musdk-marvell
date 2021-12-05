/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
