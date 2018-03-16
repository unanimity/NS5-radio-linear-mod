/*
 * play_list.c
 *
 * Created: 20.01.2016 11:44:00
 *  Author: Anton
 */ 

#include "play_list.h"
#include <string.h>

//возвращает имя файла по номеру в name, 
//ret - номер файла или 0 если нет такого имени
uint8_t playlist_getfilename(uint8_t num, char *name)
{
	#define		case_file_name(suf, unused)		case suf: \
													strcpy_P(name, PSTR(ATPASTE2(FILE_NAME_, suf))); \
												break;
										
	switch (num)
	{
		MREPEAT (PLAY_LIST_NUM, case_file_name, ~)
		
		default:
			num = 0;
		break;
	}
	name[0] = LUN_ID_SD_MMC_0_MEM + '0';
	return num;
}

void playlist_init(play_list_t *list)
{
	#define		file_pause_fill(suf, unused)	list->pause[suf] = ATPASTE2(FILE_PAUSE_, suf); \
												list->file_num[suf] = suf;
	MREPEAT (PLAY_LIST_NUM, file_pause_fill, ~)	
}