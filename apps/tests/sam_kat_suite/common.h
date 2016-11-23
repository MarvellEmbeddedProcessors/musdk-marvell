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
