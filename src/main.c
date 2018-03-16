/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <string.h>
#include <util/atomic.h>
#include "usart_driver.h"
#include "ad9954.h"
#include "pcm.h"
#include "DAC.h"
#include "morse.h"
#include <util/delay.h>
#include "play_list.h"

//RTC
volatile uint32_t rtc_tic;
volatile uint16_t local_tic;

#define reset_local_tic  ATOMIC_BLOCK(ATOMIC_FORCEON) \
{	\
	local_tic = 0; \
}
#define comp_local_tic(D)  (local_tic == D)

#define	player_start()		TCC0.CTRLA = TC_CLKSEL_DIV1_gc; 
#define	player_stop()		TCC0.CTRLA = TC_CLKSEL_OFF_gc;

#define sampleNum	64
#define		ftw_buf_reg			GPIOR0		//
#define		FTW_BUF_FLAG		1<<0		//0 - текущий буфер ftw_buf[0]
											//1 - текущий буфер ftw_buf[1]
#define		PLAY_STOP			1<<1		//0 - воспроизведение
											//1 - текущий семпл последний, остановить воспроизведение

#define		BUF_INUSE			1<<0
#define		BUF_READY			1<<1


//FS
	char file_name[32];
	Ctrl_status status;
	FRESULT res;
static	FATFS fs;
static	FIL file_object;
static	WAVHEADER_t wavheader;



typedef  struct
	{	uint32_t				ftw0[sampleNum];	//данные частоты
		uint32_t                ftw1[sampleNum];    //данные частоты 1
		uint8_t                 LSCW[sampleNum][5];    //данные прирощения
		uint8_t                 PS0[sampleNum];;    //+- дельта
		volatile int16_t		FTW_old;
		uint8_t					header;	//текущая позиция воспроизведения
		volatile uint8_t		status;		//использование буфера
		
	} ftw_buf_t;
    
volatile int16_t		FTW_tmp; // костыль 
ftw_buf_t	*ftw_buf_current;
ftw_buf_t	*fill_ftw_buf;
ftw_buf_t	ftw_buf[2];

//uint32_t	ftw0_reg;

int16_t	sampledata[sampleNum];

//состояния планировщика сообщений и работы с SD
typedef enum 
	{
		mmc_check_status = 0,			//проверка и ожидание готовности карточки
		mmc_mount,				//монтирование карты и файловой системы
		play_sound,				//воспроизведение сообщения
		play_telemetry,			//воспроизведение телеметрии
		play_CW,				//CW
		play_sheduler,			//чтение списка воспроизведения		
	} playlist_state_t;

playlist_state_t	playlist_state;
play_list_t			play_list;

uint8_t			play_list_element;				//текущий номер
//состояние сообщения
typedef enum
	{
		stop = 0,			//ничего не передаем, после смены состояний
		playback,			//воспроизведение
		end,				//закончили воспроизводить
	} play_state_t;

play_state_t play_state;
//alarm flag
volatile bool alarm;

//RTC
static inline void init_RTC(void)
{
	OSC.XOSCCTRL  = OSC_XOSCSEL_32KHz_gc;
	OSC.CTRL |= OSC_XOSCEN_bm;
	while (!(OSC.STATUS & OSC_XOSCRDY_bm));
	
	CLK.RTCCTRL = CLK_RTCSRC_TOSC_gc | CLK_RTCEN_bm;

	
	sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_RTC);
	RTC.PER = 0xffff;
	RTC.CNT = 0;
	/* Since overflow interrupt is needed all the time we limit sleep to
	 * power-save.
	 */
	sleepmgr_lock_mode(SLEEPMGR_PSAVE);
	RTC.INTCTRL = RTC_COMPINTLVL_LO_gc;
	RTC.CTRL = CONFIG_RTC_PRESCALER;
}

static inline void init_AUTOCAL (void)
{
	if (OSC.STATUS & OSC_XOSCRDY_bm)
	{
		OSC.DFLLCTRL = OSC_RC32MCREF_XOSC32K_gc;
		DFLLRC32M.CTRL = DFLL_ENABLE_bm;
	}
}




void mmc_usartspi_init (void)
{
	//spi_master_init(&SPI_master_module);
	//spi_master_setup_device(&SPI_master_module, &SPI_DEVICE_CS0, SPI_MODE_0, 1000000, 0);
	//spi_enable(&SPI_master_module);
	usart_spi_init(DDS_SPI);
	usart_spi_setup_device(DDS_SPI, DDS_SPI_0_CS, SPI_MODE_0, 10000000, 0);
	usart_spi_enable(DDS_SPI);
}

//таймер для обновления мгновенной частоты 8 кГц, 125мкс (sample rate)
void timer_init(void)
{
	sysclk_enable_module(SYSCLK_PORT_C, PR_TC0_bm);
	TCC0.INTCTRLA = TC_OVFINTLVL_LO_gc;
	TCC0.PER = 4000;
}

FRESULT sample_read (FIL* file, uint8_t* data, UINT size, UINT* realread)			/* Read data from a file */
{
	uint8_t p;
	uint8_t tmp, r;
	FRESULT res;
	for (p = 0; p < size; p++)
	{
		res = f_read(file, &tmp, 1, &r);
		if (FR_OK == res)
		{
			data[p] = tmp;
			data++;
		}
		else
		{
			*realread = p;
			return res;
		}
	}
	*realread = p;
	return FR_OK;
}


void setLCS(uint8_t *in, uint8_t *out)
{
	out[0] = 100;
	out[1] = in[3];
	out[2] = in[2];
	out[3] = in[1];
	out[4] = in[0];


}
void reversbyte4(uint8_t *in, uint8_t *out)
{
	out[0] = in[3];
	out[1] = in[2];
	out[2] = in[1];
	out[3] = in[0];
}
// 

//преобразование семпла 16бит в данные частоты для DDS 32 бита, размер сампла в словах
void sample2ftw(int16_t *sample, ftw_buf_t *ftw_buf, uint16_t len)
{	uint16_t tmplen;
	int16_t F_k1;
	int16_t F_k=0;
	uint32_t dds_F_k1;
	uint32_t dds_F_k;
    int32_t delta;
	uint64_t tmp_del;
    uint32_t ramp;  
	uint16_t i = 0;
	uint16_t j = 0;
	tmplen=len;
	while(len)
	{
	 //-------------------------
	 if (i==0) F_k=FTW_tmp; 
	 F_k1 = sample[i];
	 {
	 if (F_k1 >= 0)
	 {
		 dds_F_k1 = DDS_CARRIER_FREQ + F_k1;
	 }
	 else
	 {
		 dds_F_k1 = DDS_CARRIER_FREQ - abs(F_k1);
	 }}
	 //-----------
	 {
		 if (F_k >= 0)
		 {
			 dds_F_k = DDS_CARRIER_FREQ + F_k;
		 }
		 else
		 {
			 dds_F_k = DDS_CARRIER_FREQ - abs(F_k);
		 }
	}
	//------------------------
		
		 if ((dds_F_k<dds_F_k1))

		 {	 delta=dds_F_k1-dds_F_k;
			 reversbyte4(&dds_F_k, &ftw_buf->ftw0[j]);
			 reversbyte4(&dds_F_k1,&ftw_buf->ftw1[j]);
			 tmp_del=delta*16384;// 0,093166667 * 2^14
			 ramp=tmp_del>>14;
			 // запись ската
			 //setLCS(&ramp, &ftw_buf->LSCW[j]);
			 ftw_buf->PS0[j]=1;

		 }
		 
		 else
		 
		 {
			 delta=dds_F_k-dds_F_k1;
			 reversbyte4(&dds_F_k, &ftw_buf->ftw1[j]);
			 reversbyte4(&dds_F_k1,&ftw_buf->ftw0[j]);
			 tmp_del=delta*16384;// 0,093166667 * 2^14
			 ramp=tmp_del>>14;
			 
			// запись ската
			// setLCS(&ramp, &ftw_buf->LSCW[j]);
			 ftw_buf->PS0[j]=0;
		 }




		F_k=F_k1;//
		i++;
		j++;
		len--;
	} 
	
	FTW_tmp=F_k1;
}

//тестовое заполнение для ЦАП
void sample2ftw_dac(int16_t *sample, uint32_t *ftw_buf, uint16_t len)
{
	int16_t sample_data;
	uint32_t dds_data;
	uint16_t i = 0;
	uint16_t j = 0;
	while(len)
	{
		sample_data = sample[i];
		if (sample_data >= 0)
		{
			dds_data = 2048 + sample_data / 16;
		}
		else
		{
			dds_data = 2048 - sample_data / 16;
		}
		ftw_buf[j] = dds_data;
		i++;
		j++;
		len--;
	}
}


void flip_ftw_buf(void)
{
//	irqflags_t flags = cpu_irq_save();
	if (ftw_buf_reg & FTW_BUF_FLAG)
	{
		//был буфер 1
		ftw_buf_current = &ftw_buf[0];
		ftw_buf_current ->FTW_old=ftw_buf[1].FTW_old;
		ftw_buf_reg &= ~FTW_BUF_FLAG;
	} 
	else
	{
		//был буфер 0
		ftw_buf_current = &ftw_buf[1];
		ftw_buf_current ->FTW_old=ftw_buf[0].FTW_old;
		ftw_buf_reg |= FTW_BUF_FLAG;
	}
//	cpu_irq_restore(flags);
}

void PTT_on (void)
{
	//
	LED_ON(LED2);
	//DAC_set(1550);
	//DAC_set(PA_LEVEL_1W);
	ioport_set_pin_high(EN_VPRE_GPIO);
	dds_run();
}

void PTT_off (void)
{
	LED_OFF(LED2);
	dds_stop();
	ioport_set_pin_low(EN_VPRE_GPIO);
	DAC_set(0);
}

void rtc_callback(uint32_t time)
{
	alarm = true;
}

int main (void)
{
//	char test_file_name[] = "0:sd_mmc_test_ver2.txt";
//	char audio_file_name[] = "0:The_Unforgiven_II.wav";
//	char audio_folder[] = "Audio";
		
	uint8_t flag = 0;
	
	
	UINT		byteread;
	bool		file_end;
	
	uint16_t	length_sec;
	uint32_t	tmp;

	//uint32_t	tmp_freq;
	//uint32_t	tmp_ftw;
	uint16_t	i, j;
	
	uint32_t	alarm_time;	

	uint8_t		 mmc_check_count;
	
	/* Insert system clock initialization code here (sysclk_init()). */
	sysclk_init();
	rtc_init();
	board_init();
	timer_init();
	USART_Init();
	init_RTC();
	init_AUTOCAL();
	
	irq_initialize_vectors();
	cpu_irq_enable();
	//mmc_usartspi_init();
	sd_mmc_init();
	dds_usart_spi_init();
	dds_init();
	
	
	
	dds_stop();
	DAC_init(0);
	morse_init(20);
	morse_set_pa_level(PA_LEVEL_1W);
	ioport_set_pin_high(EN_VPA_GPIO);
	
	msgdbg("\x0C\n\r[Radio] system start \r\n");
	//while(1);
	

	alarm = false;
	rtc_set_callback(rtc_callback);
	rtc_set_alarm_relative(0);
	playlist_state = mmc_check_status;
	play_state = stop;
	
	play_list_element = 0;
	playlist_init(&play_list);
	
// 	PTT_on();
// 	tmp = DDS_CARRIER_FREQ;
// 	dds_write_4byte(FTW0_REG, &tmp);
// 	dds_update();
// 	
// 	delay_ms(3000);
// 	tmp = 0;
// 	dds_write_4byte(FTW0_REG, &tmp);
// 	dds_update();
// 	PTT_off();
// 	while(1);
	
	while(1)
	{
		if (alarm)
		{
// 			alarm = false;
// 			rtc_set_alarm_relative(0);
// 			msgdbg("Tic %lu\r\n", rtc_get_time());
 			//ioport_toggle_pin(LED1);

			switch (playlist_state)
			{
				//проверка и ожидание готовности карточки
				case mmc_check_status:
						status = sd_mmc_test_unit_ready(0);
						msgdbg("[mmc_check_status]Test mmc\r\n");
						if (CTRL_GOOD == status)
						{
							playlist_state = mmc_mount;
						}
						else
						{
							if (++mmc_check_count > 100)
							{
								playlist_state = play_CW;
								mmc_check_count = 0;
							}
							else
							{
								alarm = false;
								rtc_set_alarm_relative(1);	
							}
						}
				break;
				//монтирование карты и файловой системы
				case mmc_mount:
					msgdbg("[mmc_mount] start (f_mount)...\r\n");
					memset(&fs, 0, sizeof(FATFS));
					res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);
					if (FR_OK  == res)
					{
						msgdbg("[mmc_mount] done...\r\n");
						playlist_state = play_sheduler;
					}
					else
					{
						msgdbg("[mmc_mount] error...\r\n");
						playlist_state = mmc_check_status;
					}
				break;
				//воспроизведение сообщения
				case play_sound:
					//воспроизводим сообщение
					if (play_state == stop)
					{
						//первый семпл
						//читаем семпл и заполняем буфер
						res = f_read(&file_object, &sampledata, 2 * sampleNum, &byteread);
						if (FR_OK  == res)
						{
							if (byteread == 2 * sampleNum)
							{
								//подготавливаем буфер для воспроизведения
								ftw_buf_current = &ftw_buf[0];
								ftw_buf_reg = 0;
								sample2ftw(&sampledata, ftw_buf_current, sampleNum);
								ftw_buf_current->header = 0;
								ftw_buf_current->status = BUF_INUSE | BUF_READY;
								
								//запускаем воспроизведение
								PTT_on();
								player_start();
								play_state = playback;								
							}
							else
							{
								//закончили воспроизведение
								play_state = end;
								playlist_state = play_sheduler;
								PTT_off();
								player_stop();
							}
						}
						else
						{
							play_state = stop;
							playlist_state = play_sheduler;
						}
					} 
					else if (play_state == playback)
					{
						//очередной семпл
						ATOMIC_BLOCK(ATOMIC_FORCEON)
						{
							if (ftw_buf_reg & FTW_BUF_FLAG)
							{
								fill_ftw_buf = &ftw_buf[0];
							}
							else
							{
								fill_ftw_buf = &ftw_buf[1];
							}	 
						}
						if (!(fill_ftw_buf->status & BUF_INUSE) && !(fill_ftw_buf->status & BUF_READY))
						{
							res = f_read(&file_object, &sampledata, 2 * sampleNum, &byteread);
							if (FR_OK  == res)
							{
								if (byteread == (2 * sampleNum))
								{
									//подготавливаем буфер для воспроизведения
									sample2ftw(&sampledata, fill_ftw_buf, sampleNum);
									fill_ftw_buf->header = 0;
									fill_ftw_buf->status = BUF_READY;
								}
								else if (byteread > 0)
								{
									//последний некратный семпл
									//подготавливаем буфер для воспроизведения
									for (uint8_t count = byteread; count < sampleNum; count++)
									{
										sampledata[count] = 0;
									}
									sample2ftw(&sampledata, fill_ftw_buf, sampleNum);
									msgdbg(" F= %d\r\n", fill_ftw_buf->PS0[5]);
									fill_ftw_buf->header = 0;
									fill_ftw_buf->status = BUF_READY;
								}
								else
								{
									//закончили воспроизведение
									play_state = end;
									playlist_state = play_sheduler;
									PTT_off();
									player_stop();
								}
							}
							else
							{
								msgdbg("[play_sound] error reading file %s\r\n", file_name);
								play_state = end;
								playlist_state = play_sheduler;
								PTT_off();
								player_stop();
							}
							
						}
					}
				break;
				//воспроизведение телеметрии
				case play_telemetry:
				
				break;
				//CW
				case play_CW:
				
				break;
				//чтение списка воспроизведения
				case play_sheduler:
					switch (play_state)
					{
						case stop:
							//начинаем воспроизводить очередной элемент
							if ((play_list.file_num[play_list_element] >= 0) && (play_list.file_num[play_list_element] <= PLAY_LIST_NUM))
							{
								playlist_getfilename(play_list.file_num[play_list_element], &file_name);
								
								res = f_open(&file_object,
											(char const *)file_name,
											FA_OPEN_EXISTING | FA_READ);
								if (FR_OK  == res)
								{
									msgdbg("[play_sheduler] start playback file %d: ", play_list.file_num[play_list_element]);
									msgdbg("%s\r\n", file_name);
									//проверяем корректность фала звукового
									res = f_read(&file_object,
												&wavheader,
												sizeof(WAVHEADER_t),
												&byteread);
									if (FR_OK  == res)
									{
										if (wavheader.audioFormat == 1 &&
											wavheader.numChannels == 1 &&
											wavheader.sampleRate == 8000 &&
											wavheader.bitsPerSample == 16)
										{
											//файл соответствует
											//подсчитываем длительность звучания
											tmp = wavheader.subchunk2Size;
											tmp = tmp / 8000 / 2;	//длительность в сек
											length_sec = (uint16_t) tmp;
											msgdbg("[play_sheduler] Audio length %d s\r\n", length_sec);
											//проверяем перекрытие паузы
											if (length_sec < play_list.pause[play_list_element])
											{
												//пауза штатная
												alarm_time = rtc_get_time() + play_list.pause[play_list_element];
											}
											else
											{
												//пауза минимальная 1с
												alarm_time = rtc_get_time() + length_sec + 1;
											}
									

											playlist_state = play_sound;
											
										}
										else
										{
											play_state = end;
											msgdbg("[play_sheduler] wrong audio format\r\n");
										}
									}
									else
									{
										msgdbg("[play_sheduler] fail read frome file %s\r\n", file_name);
										playlist_state = mmc_check_status;
										#ifdef  DEBUG
											delay_ms(500);
										#endif
									}
									
								}
								else
								{
									f_close(&file_object);
									msgdbg("[play_sheduler] fail open file %s\r\n", file_name);
									#ifdef  DEBUG
										delay_ms(500);
									#endif
									playlist_state = mmc_check_status;
									
								}	
								
							} 
							else
							{
								if (play_list.file_num[play_list_element] == PLAY_TELEMETRY_CODE)
								{
									playlist_state = play_telemetry;
								} 
								else if (play_list.file_num[play_list_element] == PLAY_CW_CODE)
								{
									playlist_state = play_CW;
								}
							}
												
								
						break;
						case playback:
							
						break;
						case end:
							f_close(&file_object);
							msgdbg("[play_sheduler] playback end: %s\r\n", file_name);
							//LED_OFF(LED0);
							alarm = false;
							rtc_set_alarm(alarm_time);
							if (++play_list_element == PLAY_LIST_NUM)
							{
								play_list_element = 0;
							}
							play_state = stop;
						break;
						default:
						
						break;
					}
				break;
				default:
				/* Your code here */
				break;
			}
		}
	}
}


ISR(TCC0_OVF_vect)
{ioport_set_pin_low(LED1);
	uint8_t	p = ftw_buf_current->header;
	uint32_t F0_tmp = ftw_buf_current->ftw0[p];
	uint32_t F1_tmp = ftw_buf_current->ftw1[p];
	uint8_t PS0_tmp =ftw_buf_current->PS0[p];
	uint8_t *LSCW =&ftw_buf_current->LSCW[p];
	 
	dds_write_dds(&F0_tmp,&F1_tmp,LSCW,LSCW,&PS0_tmp);
	dds_update();
	
	if (PS0_tmp)
	ioport_set_pin_high(DDS_PS0);
	else
	ioport_set_pin_low(DDS_PS0);
	
	if (p >= (sampleNum - 1))
	{
		//конец буфера
		ftw_buf_current->status = 0;
		flip_ftw_buf();
		ftw_buf_current->status |= BUF_INUSE;
		ioport_toggle_pin(LED0);			
	}
	else
	{
		//продолжаем текущий буфер
		ftw_buf_current->header = p + 1;
	}
	//	_delay_loop_1(100);
	ioport_set_pin_high(LED1);
}

