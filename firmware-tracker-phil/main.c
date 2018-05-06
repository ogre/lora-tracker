#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern void initialise_monitor_handles(void);

#include "radio.h"
#include "cmp.h"
#include "util.h"
#include "config.h"

#ifdef CUTDOWN
  #include "../cutdownpwd.h"
  /* The file needs to declare:
    const char cutdown_text[] = "CUTDOWNpassword";
  */
#endif

#if GPS_UBLOX_VERSION == 7
  #define GPS_NAVPVT_LEN	84
#elif GPS_UBLOX_VERSION == 8
  #define GPS_NAVPVT_LEN	92
#else
  #error "You must select Ublox version (7/8) for NAV-PVT message."
#endif

#define TOTAL_SENTENCES (sizeof(sentences_coding)/sizeof(uint8_t))

#define ABS(x)	((x < 0) ? -x : x)

void init(void);
void calibrate_hsi(void);
void _delay_ms(const uint32_t delay);
void uart_send_blocking_len(uint8_t *buff, uint16_t len);
uint16_t process_packet(char* buffer, uint16_t len, uint8_t format);
uint8_t find_diff_scaling(void);
uint8_t find_diff_scaling_alt(void);
void process_diffs(uint8_t scaling_factor,uint8_t scaling_factor_alt);
void process_diff(uint8_t scaling_factor, int32_t start_val, int32_t* input_diff, int8_t* output_diff);

static uint8_t sentence_counter = 0;

char buff[150] = {0};

char gnss_buff[255] = {0};

//len = 44
const uint8_t flight_mode[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
		0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,
		0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4D, 0xDB};

//len = 8
const uint8_t flight_mode_poll[] = {0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84};

//len = 16
const uint8_t disable_nmea_gpgga[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23};
const uint8_t disable_nmea_gpgll[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};
const uint8_t disable_nmea_gpgsa[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
const uint8_t disable_nmea_gpgsv[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};
const uint8_t disable_nmea_gprmc[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F};
const uint8_t disable_nmea_gpvtg[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};

//len = 16
const uint8_t enable_navpvt[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00,
		0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1};
//len = 14
const uint8_t set_rate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xF4, 0x01, 0x01, 0x00,
		0x00, 0x00, 0x0A, 0x75};


volatile uint16_t gnss_string_count = 0; //0 - not in string, >=1 - in string
volatile uint16_t gnss_string_len = 0;
volatile uint16_t gnss_message_id = 0;
volatile char* gnss_buff_ptr = &gnss_buff[0];

volatile uint8_t pos_updated = 0;
volatile uint8_t gnss_status_updated = 0;

volatile uint8_t fixtype = 0;
volatile int32_t latitude = 0;
volatile int32_t longitude = 0;
volatile int32_t altitude = 0;
volatile uint8_t hour = 0;
volatile uint8_t minute = 0;
volatile uint8_t second = 0;
volatile uint8_t sats = 0;
volatile uint8_t pos_valid = 0;
volatile uint8_t time_valid = 0;

volatile uint16_t ms_countdown = 0;

uint16_t payload_counter = 0;
uint16_t uplink_counter = 0;
uint16_t cutdown_counter = 0;
uint8_t cutdown_status = 0;

#ifdef HABPACK
//#define HB_BUF_LEN 100
//uint8_t hb_buf[HB_BUF_LEN] = {0};
uint8_t hb_buf_ptr = 0;

static size_t file_writer(cmp_ctx_t *ctx, const void *data, size_t count) {

	uint16_t i;
	//if (hb_buf_ptr+count > HB_BUF_LEN)
	//	return -1;

	for (i = 0; i < count; i++)
	{
		((char*)ctx->buf)[hb_buf_ptr] = *((uint8_t*)data);
		data++;
		hb_buf_ptr++;
	}
	return count;
    //return fwrite(data, sizeof(uint8_t), count, (FILE *)ctx->buf);
}
#endif

static void init_wdt(void)
{
	RCC_CSR |= 1;   //LSI on
	while(RCC_CSR&(1<<1));  //wait for LSI ready

	while(IWDG_SR&1);
	IWDG_KR = 0x5555;
	IWDG_PR = 0b110; // 40kHz/256

	while(IWDG_SR&2);
	IWDG_KR = 0x5555;
	IWDG_RLR = 2600;

	IWDG_KR = 0xAAAA;
	IWDG_KR = 0xCCCC;

	/*
	///// Configuring the IWDG when the window option is disabled
	//1. Enable register access by writing 0x0000 5555 in the IWDG_KR register.
	IWDG_KR = 0x5555;
	//2. Write the IWDG prescaler by programming IWDG_PR from 0 to 7.
	IWDG_PR = 0b110; // 40kHz/256
	//3. Write the reload register (IWDG_RLR).
	IWDG_RLR = 1600;
	//4. Wait for the registers to be updated (IWDG_SR = 0x0000 0000).
	while(IWDG_SR&1)
	{
		int t = IWDG_SR;
		t++;
	}
	//5. Refresh the counter value with IWDG_RLR (IWDG_KR = 0x0000 AAAA).
	IWDG_KR = 0xAAAA;
	//6. Enable the IWDG by writing 0x0000 CCCC in the IWDG_KR.
	IWDG_KR = 0xCCCC; */
}

static void gnss_configure(void)
{
	uint16_t ubloxcrc;

	/* Configure flight mode */
	uart_send_blocking_len((uint8_t*)flight_mode,44);

	/* Disable NMEA Outputs */
	uart_send_blocking_len((uint8_t*)disable_nmea_gpgga,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpgll,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpgsa,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpgsv,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gprmc,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpvtg,16);

	/* Set GPS Nav Rate */
	memcpy(buff, set_rate,12);
	buff[6] = GPS_UPDATE_PERIOD & 0xFF;
	buff[7] = (GPS_UPDATE_PERIOD >> 8) & 0xFF;
	ubloxcrc = calculate_ublox_crc((uint8_t*)&buff[2],10);

	uart_send_blocking_len((uint8_t*)buff,12);
	usart_send_blocking(USART1,ubloxcrc>>8);
	usart_send_blocking(USART1,ubloxcrc&0xFF);

	/* Enable UBX-NAV-PVT Output */
	ubloxcrc = calculate_ublox_crc((uint8_t *)&enable_navpvt[2],16-4);

	uart_send_blocking_len((uint8_t*)enable_navpvt,14);
	usart_send_blocking(USART1,ubloxcrc>>8);
	usart_send_blocking(USART1,ubloxcrc&0xFF);
}

void init (void)
{
	rcc_clock_setup_in_hsi_out_48mhz();
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_TIM14);

	//gpio
	rcc_periph_clock_enable(RCC_GPIOF);
	gpio_mode_setup(GPIOF, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);

	//systick
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(48000-1);		//1kHz at 8MHz clock
	systick_interrupt_enable();
	systick_counter_enable();

	RCC_CR &= ~(0x1F<<3); //trim the crystal down a little
	RCC_CR |= 15<<3;

	//adc
	rcc_periph_clock_enable(RCC_ADC);
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
	uint8_t channel_array[] = { 0x1 }; // ADC Channel 1
	adc_power_off(ADC1);
	adc_calibrate(ADC1);
	//adc_set_operation_mode(ADC1, ADC_MODE_SCAN); //adc_set_operation_mode(ADC1, ADC_MODE_SCAN_INFINITE);
	adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
//	adc_set_single_conversion_mode(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_239DOT5);
	adc_set_regular_sequence(ADC1, 1, channel_array);
	adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
	//adc_set_single_conversion_mode(ADC1);
	adc_disable_analog_watchdog(ADC1);
	adc_power_on(ADC1);

	//uart
	nvic_enable_irq(NVIC_USART1_IRQ);
	rcc_periph_clock_enable(RCC_USART1);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO2 | GPIO3);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO2 | GPIO3);

	usart_set_parity(USART1 ,USART_PARITY_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX );
	usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_databits(USART1, 8);
	usart_set_baudrate(USART1, 9600);
	usart_enable_rx_interrupt(USART1);
	usart_enable(USART1);

	adc_start_conversion_regular(ADC1);

	//used to hiz the uart so the pc can query flight mode
	//gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
	//			GPIO2 | GPIO3);
}

void uart_send_blocking_len(uint8_t *_buff, uint16_t len)
{
	uint16_t i = 0;
	for (i = 0; i < len; i++)
		usart_send_blocking(USART1,*_buff++);

}

void sys_tick_handler(void)
{
	if (ms_countdown)
		ms_countdown--;
}

void usart1_isr(void)
{

	if (((USART_ISR(USART1) & USART_ISR_RXNE) != 0))
	{
		uint8_t d = (uint8_t)USART1_RDR;

		if (gnss_string_count == 0){ //look for '0xB5'
			if (d == 0xB5){
				gnss_string_count++;
			}
		}
		else if (gnss_string_count == 1){ //look for '0x62'
			if (d == 0x62)
				gnss_string_count++;
			else if (d == 0xB5)
				gnss_string_count = 1;
			else
				gnss_string_count = 0;
		}
		else if (gnss_string_count == 2){  //message id
			gnss_message_id = d << 8;
			gnss_string_count++;
		}
		else if (gnss_string_count == 3){  //message id
			gnss_message_id |= d;
			if (gnss_message_id != 0x0107)
				gnss_string_count = 0;  //not looking for this string, reset
			else
				gnss_string_count++;
		}
		else if (gnss_string_count == 4){  //length top byte
			gnss_string_len = d;
			gnss_string_count++;
		}
		else if (gnss_string_count == 5){  //length bottom byte
			gnss_string_len |= d << 8;
			gnss_string_count++;
			gnss_buff_ptr = &gnss_buff[0];
		}
		else if (gnss_string_count > 0){  //process payload + checksum
			gnss_string_count++;

			if (gnss_string_count < sizeof(gnss_buff))
				*gnss_buff_ptr++ = d;
			else
				gnss_string_count = 0;   //something probably broke

			//if (gnss_string_count >= 256){
			//	gnss_string_count = 0;   //something probably broke
			//}

			if ((gnss_string_count-6-2) == gnss_string_len) //got all bytes, check checksum
			{
				//lets assume checksum == :)
				if ((gnss_message_id == 0x0107) && (gnss_string_len == GPS_NAVPVT_LEN))  //navpvt
				{
					fixtype = gnss_buff[20];
					uint8_t valid_time = gnss_buff[11];  //valid time flags

					if (fixtype == 2 || fixtype == 3){
						latitude = (gnss_buff[31] << 24)
								 | (gnss_buff[30] << 16)
								 | (gnss_buff[29] << 8)
								 | (gnss_buff[28]);
						longitude = (gnss_buff[27] << 24)
								 | (gnss_buff[26] << 16)
								 | (gnss_buff[25] << 8)
								 | (gnss_buff[24]);
						altitude = (gnss_buff[39] << 24)
								 | (gnss_buff[38] << 16)
								 | (gnss_buff[37] << 8)
								 | (gnss_buff[36]);
						pos_valid |= 1;
					}
					if (valid_time & (1<<1))
					{
						hour = gnss_buff[8];
						minute = gnss_buff[9];
						second = gnss_buff[10];
						time_valid |= 1;
					}

					sats = gnss_buff[23];
					gnss_status_updated = 1;
					pos_updated = 1;
				}

				gnss_string_count = 0;  //wait for the next string
			}
		}
	}
	else// if (((USART_ISR(USART1) & USART_ISR_ORE) != 0))  //overrun, clear flag
	{
		USART1_ICR = USART_ICR_ORECF;
	}
	//else //clear all the things
	//{
	//	USART1_ICR = 0x20a1f;
	//}
}

int main(void)
{
#ifdef HABPACK
	uint16_t k;
	radio_lora_settings_t s_lora;
#ifdef UPLINK
 	uint8_t uplink_en = 1;
#endif
#endif

	init();

#ifndef TESTING
	init_wdt();
#endif

	_delay_ms(200);
	gnss_configure();
	radio_init();

	_delay_ms(100);
 	radio_high_power();
	radio_set_frequency_frreg(RADIO_FREQ);

	while(1)
	{

#ifndef ENABLE_GPS
		gnss_status_updated = 1;
#endif

		/* Set Flight Mode */
		uart_send_blocking_len((uint8_t*)flight_mode,44);

#ifdef CUTDOWN
		cutdown_status = gpio_get(GPIOF,GPIO1) > 0;
#endif

		ms_countdown = 1000;
		while((pos_updated == 0) && (gnss_status_updated == 0) && (ms_countdown)) {};

		if((pos_updated == 0) && (gnss_status_updated == 0))
		{
			/* Didn't receive position, so reconfigure GNSS */
			gnss_configure();
			/* then transmit old info anyway */
		}

		//WDT reset
		IWDG_KR = 0xAAAA;

		sentence_counter++;
		if (sentence_counter >= TOTAL_SENTENCES)
			sentence_counter = 0;

		radio_sleep();
		_delay_ms(10);

#ifdef UPLINK
		uplink_en = 0;
#endif
		if (sentences_bandwidth[sentence_counter] == RTTY_SENTENCE)
		{
            /* RTTY & ASCII */
			process_packet(buff,100,0);

			radio_high_power();
			radio_start_tx_rtty((char*)buff,BAUD_50,4);

			while(rtty_in_progress() != 0){
				radio_rtty_poll_buffer_refill();
				_delay_ms(20);
			}
			/* RTTY Postamble */
			_delay_ms(300);
			radio_sleep();
		}
#ifdef HABPACK
#ifdef CALLING
		else if((payload_counter % CALLING_INTERVAL) == 0)
		{
			/* LoRa & HABpack Calling Mode */
			s_lora.spreading_factor = 11;
			s_lora.bandwidth = BANDWIDTH_41_7K;
			s_lora.coding_rate = CODING_4_8;
			s_lora.implicit_mode = 0;
			s_lora.crc_en = 1;
			s_lora.low_datarate = 0;

			k=process_packet(buff,100,2);

			radio_write_lora_config(&s_lora);

			radio_standby();
			radio_high_power();
			radio_set_frequency_frreg(CALLING_FREQ);

			radio_tx_packet((uint8_t*)(&buff[0]),k);

			_delay_ms(200);

			while(lora_in_progress())
				_delay_ms(50);
		}
#endif
		else
		{
            /* LoRa & HABpack */
			s_lora.spreading_factor = sentences_spreading[sentence_counter];
			s_lora.bandwidth = sentences_bandwidth[sentence_counter];
			s_lora.coding_rate = sentences_coding[sentence_counter];
			s_lora.implicit_mode = sentences_implicit[sentence_counter];
			s_lora.crc_en = 1;
			s_lora.low_datarate = 1;

#ifdef UPLINK
            if (s_lora.bandwidth != BANDWIDTH_125K)
                uplink_en = 1;
#endif

			k=process_packet(buff,100,1);


			radio_write_lora_config(&s_lora);

			radio_standby();
			radio_high_power();
			radio_set_frequency_frreg(RADIO_FREQ);

			radio_tx_packet((uint8_t*)(&buff[0]),k);

			_delay_ms(200);

			while(lora_in_progress())
				_delay_ms(50);
		}
#endif

#ifdef UPLINK

		if (uplink_en)// && !((payload_counter & 0x3) == 0x3))
		{
			radio_sleep();
			s_lora.bandwidth = BANDWIDTH_62_5K;
			s_lora.spreading_factor = 12;
			radio_write_lora_config(&s_lora);
			radio_standby();
			radio_pa_off();
			radio_lna_max();
			radio_set_frequency_frreg(RADIO_FREQ);
			radio_write_single_reg(REG_IRQ_FLAGS,0xFF);
			radio_set_continuous_rx();

			//see if we get a header
			_delay_ms(400);
			uint8_t stat = radio_read_single_reg(REG_MODEM_STAT);
			if (stat & (1<<0))
			{
				//wait for packet
				uint8_t count = 100;
				_delay_ms(300);
				while(count){
					_delay_ms(40);
					uint8_t irq = radio_read_single_reg(REG_IRQ_FLAGS);
					if (irq & (1<<6))
					{
						count = 0;
						uplink_counter++;
#ifdef CUTDOWN
						//get message - reuse tx buffer
						int16_t r = radio_check_read_rx_packet(sizeof(buff)/sizeof(char),(uint8_t*)buff,1);
						if (r > 0)
						{
							//parse uplinked text...
							uint8_t cut = 1;
							uint8_t cmp_ptr;
							if ( r == (sizeof(cutdown_text)/sizeof(char))){
								for (cmp_ptr = 0; cmp_ptr < r; cmp_ptr++){
									if (buff[cmp_ptr] != cutdown_text[cmp_ptr])
										cut = 0;
								}
								if (cut){
									cutdown_counter++;
									gpio_mode_setup(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);  //FIRE!
									gpio_clear(GPIOF,GPIO1);
									_delay_ms(2000);
									gpio_mode_setup(GPIOF, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);

								}
							}

						}
#endif
					}
					else
						count--;
				}
			}

			radio_sleep();
			_delay_ms(10);
		}
		else
			_delay_ms(50);
#else
		_delay_ms(50);
#endif
	}
}

//returns length written
//format - 0 = UKHAS ASCII
//         1 = HABpack
//         2 = HABpack Calling Beacon
uint16_t process_packet(char* buffer, uint16_t len, uint8_t format)
{
	uint16_t k;

	/* Copy out GPS data, disable IRQ while we do */
	nvic_disable_irq(NVIC_USART1_IRQ);
	int32_t _latitude = latitude;
	int32_t _longitude = longitude;
	int32_t _altitude = altitude/1000;
	uint8_t _hour = hour;
	uint8_t _minute = minute;
	uint8_t _second = second;
	uint8_t _sats = sats;
	gnss_status_updated = 0;
	pos_updated = 0;
	gnss_status_updated = 0;
	nvic_enable_irq(NVIC_USART1_IRQ);

	/* Read Battery Voltage */
	uint32_t bv = ADC1_DR;
	bv = bv * BATTV_MUL;
	bv = bv / BATTV_DIV;
	adc_start_conversion_regular(ADC1);

	if(format == 0)
    {
        /* ASCII RTTY */
		uint16_t crc;

		k = 0;
		/* Mysterious header */
		buff[k++] = 0x55;
		buff[k++] = 0xAA;
		buff[k++] = 0x55;
		buff[k++] = 0x80;
		buff[k++] = 0x80;
		buff[k++] = 0x80;
		buff[k++] = 0x80;

#ifdef TESTING
		k+=snprintf(&buffer[k],len-k,"$$PAYIOAD,%u,",payload_counter++);
#else
		k+=snprintf(&buffer[k],len-k,"$$%s,%u,",CALLSIGN_STR,payload_counter++);
#endif
		if (time_valid)
			k+=snprintf(&buffer[k],len-k,"%02u:%02u:%02u,",
					_hour,_minute,_second);
		else
			k+=snprintf(&buffer[k],len-k,",");

		if (pos_valid)
			k+=snprintf(&buffer[k],len-k,"%ld.%07ld,%ld.%07ld,%ld,%u",
					_latitude / 10000000, ABS(_latitude) % 10000000,
					_longitude / 10000000, ABS(_longitude) % 10000000,
					_altitude,
					_sats);
		else
			k+=snprintf(&buffer[k],len-k,",,,%u",
					_sats);

		k+=snprintf(&buffer[k],len-k,",%ld.%03ld",
					bv / 1000, bv % 1000);

#ifdef UPLINK
		k+=snprintf(&buffer[k],len-k,",%u",uplink_counter);
#ifdef CUTDOWN
		k+=snprintf(&buffer[k],len-k,",%u",cutdown_counter);
		if (cutdown_status)
			k+=snprintf(&buffer[k],len-k,",OK");
		else
			k+=snprintf(&buffer[k],len-k,",ERR");
#endif
#endif
		crc = calculate_crc16(&buffer[9]);

		k+=snprintf(&buffer[k],15,"*%04X\n",crc);

		/* Mysterious footer */
		buffer[k++] = 0x80;
		buffer[k++] = 0x80;
		buffer[k] = '\0';

        return k;
	}
#ifdef HABPACK
	else if (format == 1)
	{
        /* HABpack */
		memset((void*)buffer,0,len);
		hb_buf_ptr = 0;

		cmp_ctx_t cmp;
		hb_buf_ptr = 0;
		cmp_init(&cmp, (void*)buffer, 0, file_writer);

		uint8_t total_send = 6;
#ifdef UPLINK
		total_send += 1;
#endif

		cmp_write_map(&cmp,total_send);

		cmp_write_uint(&cmp, 0);
#ifdef TESTING
		cmp_write_str(&cmp, "PAYIOAD", 7);
#else
#ifdef CALLSIGN_INT
		cmp_write_sint(&cmp, CALLSIGN_INT);
#else
		cmp_write_str(&cmp, CALLSIGN_STR, (sizeof(CALLSIGN_STR)/sizeof(char))-1);
#endif
#endif

		cmp_write_uint(&cmp, 1);
		cmp_write_uint(&cmp, payload_counter++);

		cmp_write_uint(&cmp, 2);
		cmp_write_uint(&cmp, (uint32_t)_hour*(3600) + (uint32_t)_minute*60 + (uint32_t)_second);

		cmp_write_uint(&cmp, 3);
		cmp_write_array(&cmp, 3);
		cmp_write_sint(&cmp, _latitude);
		cmp_write_sint(&cmp, _longitude);
		cmp_write_sint(&cmp, _altitude);

		cmp_write_uint(&cmp, 4);
		cmp_write_uint(&cmp, _sats);

		cmp_write_uint(&cmp, 6);
		cmp_write_sint(&cmp, bv);

#ifdef UPLINK
		cmp_write_uint(&cmp, 30);
		cmp_write_uint(&cmp, uplink_counter);
#endif
		return hb_buf_ptr;
	}
#ifdef CALLING
	else if (format == 2)
	{
		/* HABpack Calling Beacon */
		memset((void*)buffer,0,len);
		hb_buf_ptr = 0;

		cmp_ctx_t cmp;
		hb_buf_ptr = 0;
		cmp_init(&cmp, (void*)buffer, 0, file_writer);

		uint8_t total_send = 3;
#ifndef CALLING_DOWNLINK_MODE
		total_send = 7;
#endif
		
		cmp_write_map(&cmp,total_send);

		cmp_write_uint(&cmp, 0);

#ifdef TESTING
		cmp_write_str(&cmp, "PAYIOAD", 7);
#else
#ifdef CALLSIGN_INT
		cmp_write_sint(&cmp, CALLSIGN_INT);
#else
		cmp_write_str(&cmp, CALLSIGN_STR, (sizeof(CALLSIGN_STR)/sizeof(char))-1);
#endif
#endif

		cmp_write_uint(&cmp, 20);
		cmp_write_uint(&cmp, CALLING_DOWNLINK_FREQ);

#ifdef CALLING_DOWNLINK_MODE
		cmp_write_uint(&cmp, 21);
		cmp_write_uint(&cmp, CALLING_DOWNLINK_MODE);
#else
		cmp_write_uint(&cmp, 22);
		cmp_write_uint(&cmp, CALLING_DOWNLINK_IMPLICIT);
		cmp_write_uint(&cmp, 23);
		cmp_write_uint(&cmp, CALLING_DOWNLINK_ERRORCODING);
		cmp_write_uint(&cmp, 24);
		cmp_write_uint(&cmp, CALLING_DOWNLINK_BANDWIDTH);
		cmp_write_uint(&cmp, 25);
		cmp_write_uint(&cmp, CALLING_DOWNLINK_SPREADING);
		cmp_write_uint(&cmp, 26);
		cmp_write_uint(&cmp, CALLING_DOWNLINK_LDO);
#endif
		payload_counter++;

		return hb_buf_ptr;
	}
#endif
#endif
	
    /* Format not recognised */
    return 0;
}

void _delay_ms(const uint32_t delay)
{
	ms_countdown = delay;
	while(ms_countdown) { };
}