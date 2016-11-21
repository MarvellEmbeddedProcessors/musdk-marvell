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

#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

bool isValidHexPhrase(char* str) {
	bool result = true;
	unsigned int i;
	for (i = 0; i < strlen(str); i++) {
		if (!isLittleHexLetter(str[i]) && !isBigHexLetter(str[i])
				&& !isHexNumber(str[i])) {
			result = false;
		}
	}
	return result;
}
bool isContinuesData(char* line) {
	if (!line) {
		return false;
	}
	while (*line != 0) {
		if (*line == '\\') {
			cleanStr(line);
			return true;
		}
		line++;
	}
	return false;
}
bool getPhraseInQuotationMarks(char* strOut, char* line) {
	if (!strOut || !line) {
		return false;
	}
	while (*line != 0 && *line != '"') {
		line++;
	}
	if (*line != '"') {
		return false;
	}
	line++;
	while (*line != 0 && *line != '"') {
		*strOut++ = *line++;
	}
	return *line == '"';
}
bool isBigHexLetter(char c) {
	if (c >= 'A' && c <= 'F') {
		return true;
	}
	return false;
}
int hex2dec(char c) {
	if (isLittleHexLetter(c)) {
		return c - 'a' + 10;
	} else if (isBigHexLetter(c)) {
		return c - 'A' + 10;
	}
	return c - '0';
}
int* copyIntArray(int* src, int len){
	if(!src || len <= 0){
		return NULL;
	}
	int* dst = malloc(sizeof(*dst) * len);
	if(!dst){
		return NULL;
	}
	int i;
	for (i = 0; i < len; ++i) {
		dst[i] = src[i];
	}
	return dst;
}
void removeHexHead(char* str) {
	if (!str) {
		return;
	}
	if (str[0] == '0' && str[1] == 'x') {
		int size = strlen(str) - 2;
		char strTemp[size];
		strcpy(strTemp, &str[2]);
		strcpy(str, strTemp);
	}
}
void removeBeforeColon(char* line) {
	if (!line) {
		return;
	}
	int i;
	int linelen = strlen(line);
	for (i = 0; i < linelen; i++) {
		if (line[i] == ':') {
			char tempLine[linelen - i];
			strcpy(tempLine, &line[i + 1]);
			strcpy(line, tempLine);
		}
	}
}
void cleanStr(char *var) {
	int i = 0;
	while (var[i] != '\0') {
		var[i] = '\0';
		i++;
	}
}
bool isLittleHexLetter(char c) {
	if (c >= 'a' && c <= 'f') {
		return true;
	}
	return false;
}
bool isHexNumber(char c) {
	if (c >= '0' && c <= '9') {
		return true;
	}
	return false;
}

void hexString2Int(char* str, unsigned char* outResult) {
	unsigned int i, j;
	for (i = 0, j = 0; i < strlen(str); i += 2, j++) {
		outResult[j] = hex2dec(str[i]) * 16 + hex2dec(str[i + 1]);
	}
}
void strArr2IntArr(char* str, int* array) {
	if (!str || !array) {
		return;
	}
	while (*str && *str != '"') {
		*array = *str;
		str++;
		array++;
	}
}
void strAsci2HexAsci(char* str, char* strHex) {
	if (!str || !strHex) {
		return;
	}
	while (*str && *str != '"') {
		*strHex = getHexChar(*str / 16);
		strHex++;
		*strHex = getHexChar(*str % 16);
		strHex++;
		str++;
	}
}
char getHexChar(int dacValue) {
	if (dacValue >= 10 && dacValue < 16) {
		return dacValue - 10 + 'a';
	}
	if (dacValue >= 0 && dacValue < 10) {
		return dacValue + '0';
	}
	return 0;
}
bool isEndOfBracketsExist(char *str) {
	while (*str) {
		if (*str == ')') {
			return true;
		}
		str++;
	}
	return false;
}
bool isCommentOrEmptyLine(char* line) {
	char headOfLine[CHUNK_SIZE] = { 0 };
	int scanned = sscanf(line, " %s", headOfLine);
	if (scanned == -1 || headOfLine[0] == '#') {
		return true;
	}
	return false;
}
