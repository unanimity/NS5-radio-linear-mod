/*
 * play_list.h
 *
 * Created: 20.01.2016 11:41:22
 *  Author: Anton
 */ 


#ifndef PLAY_LIST_H_
#define PLAY_LIST_H_

#include <asf.h>

#ifndef PLAY_LIST_NUM
	#define		PLAY_LIST_NUM		12			//���������� ������� � ������ ��������������
#endif

//#define		PLAY_MAX_ELEMENT	249			//���� ���-�� ����� ����� ����� ��� ��������������� 1 - 249
#define		PLAY_TELEMETRY_CODE 250			//"����� �����" � ��������� ��� �������� ����������
#define		PLAY_CALLSIGN_CODE	251			//"����� �����" � ��������� ��� �������� ���������
#define		PLAY_CW_CODE		252			//"����� �����" � ��������� ��� �������� ��������� � ���������� ����������

typedef	struct
{
	uint8_t		file_num[PLAY_LIST_NUM];	//����� �����
	uint16_t	pause[PLAY_LIST_NUM];		//����� � � ����� ������� � ��������� �������� (��������� � ������ ��� �������� ��������)
	//����� ������������� �� ������ ���������������
} play_list_t;

//#define		 FILE_NAME_0		"0:The_Unforgiven_II.wav"
#define		 FILE_NAME_0		"0:t_2000.wav"
#define		 FILE_NAME_1		"0:test2.wav"
#define		 FILE_NAME_2		"0:Dasha3.wav"
#define		 FILE_NAME_3		"0:Russian.wav"
#define		 FILE_NAME_4		"0:Portugu.wav"
#define		 FILE_NAME_5		"0:Tatar.wav"
#define		 FILE_NAME_6		"0:Kazakh.wav"
#define		 FILE_NAME_7		"0:Hindi.wav"
#define		 FILE_NAME_8		"0:German.wav"
#define		 FILE_NAME_9		"0:English.wav"
#define		 FILE_NAME_10		"0:Chinese.wav"
#define		 FILE_NAME_11		"0:Arabian.wav"

#define		 FILE_PAUSE_0		30
#define		 FILE_PAUSE_1		30
#define		 FILE_PAUSE_2		60
#define		 FILE_PAUSE_3		60
#define		 FILE_PAUSE_4		60
#define		 FILE_PAUSE_5		60
#define		 FILE_PAUSE_6		60
#define		 FILE_PAUSE_7		60
#define		 FILE_PAUSE_8		60
#define		 FILE_PAUSE_9		60
#define		 FILE_PAUSE_10		60
#define		 FILE_PAUSE_11		60



uint8_t playlist_getfilename(uint8_t num, char *name);
void playlist_init(play_list_t *list);

#endif /* PLAY_LIST_H_ */