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
int* copyIntArray(int* src, int len)
{
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
void removeHexHead(char* str)
{
	char *temp = malloc(strlen(str) + 1);

	if (!str || !temp) {
		return;
	}
	if (str[0] == '0' && str[1] == 'x') {
		strcpy(temp, &str[2]);
		strcpy(str, temp);
	}
	free(temp);
}
void removeBeforeColon(char* line)
{
	int i, linelen = strlen(line);
	char *temp = malloc(linelen + 1);

	if (!line || !temp) {
		return;
	}
	for (i = 0; i < linelen; i++) {
		if (line[i] == ':') {
			strcpy(temp, &(line[i + 1]));
			strcpy(line, temp);
			break;
		}
	}
	free (temp);
}
void cleanStr(char *var)
{
	int i = 0;
	while (var[i] != '\0') {
		var[i] = '\0';
		i++;
	}
}
bool isLittleHexLetter(char c)
{
	if (c >= 'a' && c <= 'f') {
		return true;
	}
	return false;
}
bool isHexNumber(char c)
{
	if (c >= '0' && c <= '9') {
		return true;
	}
	return false;
}

void hexString2Int(char* str, unsigned char* outResult)
{
	unsigned int i, j;
	for (i = 0, j = 0; i < strlen(str); i += 2, j++) {
		outResult[j] = hex2dec(str[i]) * 16 + hex2dec(str[i + 1]);
	}
}
void strArr2IntArr(char* str, int* array)
{
	if (!str || !array) {
		return;
	}
	while (*str && *str != '"') {
		*array = *str;
		str++;
		array++;
	}
}
void strAsci2HexAsci(char* str, char* strHex)
{
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
char getHexChar(int dacValue)
{
	if (dacValue >= 10 && dacValue < 16) {
		return dacValue - 10 + 'a';
	}
	if (dacValue >= 0 && dacValue < 10) {
		return dacValue + '0';
	}
	return 0;
}
bool isEndOfBracketsExist(char *str)
{
	while (*str) {
		if (*str == ')') {
			return true;
		}
		str++;
	}
	return false;
}
bool isCommentOrEmptyLine(char* line)
{
	char headOfLine[CHUNK_SIZE] = { 0 };
	int scanned = sscanf(line, " %s", headOfLine);
	if (scanned == -1 || headOfLine[0] == '#') {
		return true;
	}
	return false;
}
