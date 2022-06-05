/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define KEY_DDR         DDRB
#define KEY_PORT        PORTB
#define KEY_PIN         button_in
#define KEY0            0
#define KEY1            1
#define KEY2            2
#define ALL_KEYS        (1<<KEY0 | 1<<KEY1 | 1<<KEY2)
#define REPEAT_MASK     (1<<KEY1 | 1<<KEY2)       // repeat: key1, key2
#define REPEAT_START    50                        // after 500ms
#define REPEAT_NEXT     20                        // every 200ms

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3; // Debounce
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7; // Sampler

//UART_HandleTypeDef huart1;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);

void calc_data(void);
void render_lfo(void);
void adc_start(void);
void heart_beat(void);
void SystemClock_Config(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void process_buttons(void);
void print_leds(void);

volatile uint16_t log_table[256] = {
		8191,7167,6568,6143,5814,5544,5317,5119,
		4945,4790,4649,4520,4402,4293,4191,4096,
		4006,3922,3842,3766,3694,3625,3559,3497,
		3436,3378,3323,3269,3217,3167,3119,3072,
		3026,2982,2939,2898,2857,2818,2779,2742,
		2706,2670,2635,2601,2568,2536,2504,2473,
		2442,2412,2383,2354,2326,2299,2272,2245,
		2219,2193,2168,2143,2119,2095,2071,2048,
		2025,2002,1980,1958,1937,1915,1894,1874,
		1853,1833,1813,1794,1775,1756,1737,1718,
		1700,1682,1664,1646,1629,1611,1594,1577,
		1561,1544,1528,1512,1496,1480,1464,1449,
		1434,1418,1403,1389,1374,1359,1345,1331,
		1316,1302,1289,1275,1261,1248,1234,1221,
		1208,1195,1182,1169,1157,1144,1132,1119,
		1107,1095,1083,1071,1059,1047,1035,1024,
		1012,1001,990,978,967,956,945,934,
		924,913,902,892,881,871,860,850,
		840,830,819,809,799,790,780,770,
		760,751,741,732,722,713,704,694,
		685,676,667,658,649,640,631,622,
		613,605,596,587,579,570,562,553,
		545,537,529,520,512,504,496,488,
		480,472,464,456,448,440,433,425,
		417,410,402,394,387,379,372,365,
		357,350,343,335,328,321,314,307,
		300,293,286,279,272,265,258,251,
		244,237,231,224,217,210,204,197,
		191,184,178,171,165,158,152,145,
		139,133,126,120,114,108,101,95,
		89,83,77,71,65,59,53,47,
		41,35,29,23,17,12,6,0,
};

//Full Freq
volatile uint16_t tfftable[256] = {66, 81, 96, 111, 126, 141, 157, 172, 187, 202,
		217, 232, 247, 263, 278, 293, 308, 323, 338, 354,
		369, 384, 399, 414, 429, 445, 460, 475, 490, 505,
		520, 536, 551, 566, 581, 596, 611, 627, 642, 657,
		672, 687, 702, 718, 733, 748, 763, 778, 793, 809,
		824, 839, 854, 869, 884, 900, 915, 930, 945, 960,
		975, 990, 1006, 1021, 1036, 1051, 1066, 1081, 1097, 1112,
		1127, 1142, 1157, 1172, 1188, 1203, 1218, 1233, 1248, 1263,
		1279, 1294, 1309, 1324, 1339, 1354, 1370, 1385, 1400, 1415,
		1430, 1445, 1461, 1476, 1491, 1506, 1521, 1536, 1552, 1567,
		1582, 1597, 1612, 1627, 1643, 1658, 1673, 1688, 1703, 1718,
		1733, 1749, 1764, 1779, 1794, 1809, 1824, 1840, 1855, 1870,
		1885, 1900, 1915, 1931, 1946, 1961, 1976, 1991, 2006, 2022,
		2037, 2052, 2067, 2082, 2097, 2113, 2128, 2143, 2158, 2173,
		2188, 2204, 2219, 2234, 2249, 2264, 2279, 2295, 2310, 2325,
		2340, 2355, 2370, 2386, 2401, 2416, 2431, 2446, 2461, 2476,
		2492, 2507, 2522, 2537, 2552, 2567, 2583, 2598, 2613, 2628,
		2643, 2658, 2674, 2689, 2704, 2719, 2734, 2749, 2765, 2780,
		2795, 2810, 2825, 2840, 2856, 2871, 2886, 2901, 2916, 2931,
		2947, 2962, 2977, 2992, 3007, 3022, 3038, 3053, 3068, 3083,
		3098, 3113, 3129, 3144, 3159, 3174, 3189, 3204, 3219, 3235,
		3250, 3265, 3280, 3295, 3310, 3326, 3341, 3356, 3371, 3386,
		3401, 3417, 3432, 3447, 3462, 3477, 3492, 3508, 3523, 3538,
		3553, 3568, 3583, 3599, 3614, 3629, 3644, 3659, 3674, 3690,
		3705, 3720, 3735, 3750, 3765, 3781, 3796, 3811, 3826, 3841,
		3856, 3872, 3887, 3902, 3917, 3932};

volatile uint16_t tlftable[256] = {66, 70, 75, 80, 85, 90, 95, 100, 105, 109,
		114, 119, 124, 129, 134, 139, 144, 149, 153, 158,
		163, 168, 173, 178, 183, 188, 192, 197, 202, 207,
		212, 217, 222, 227, 232, 236, 241, 246, 251, 256,
		261, 266, 271, 276, 280, 285, 290, 295, 300, 305,
		310, 315, 319, 324, 329, 334, 339, 344, 349, 354,
		359, 363, 368, 373, 378, 383, 388, 393, 398, 402,
		407, 412, 417, 422, 427, 432, 437, 442, 446, 451,
		456, 461, 466, 471, 476, 481, 485, 490, 495, 500,
		505, 510, 515, 520, 525, 529, 534, 539, 544, 549,
		554, 559, 564, 568, 573, 578, 583, 588, 593, 598,
		603, 608, 612, 617, 622, 627, 632, 637, 642, 647,
		652, 656, 661, 666, 671, 676, 681, 686, 691, 695,
		700, 705, 710, 715, 720, 725, 730, 735, 739, 744,
		749, 754, 759, 764, 769, 774, 778, 783, 788, 793,
		798, 803, 808, 813, 818, 822, 827, 832, 837, 842,
		847, 852, 857, 861, 866, 871, 876, 881, 886, 891,
		896, 901, 905, 910, 915, 920, 925, 930, 935, 940,
		944, 949, 954, 959, 964, 969, 974, 979, 984, 988,
		993, 998, 1003, 1008, 1013, 1018, 1023, 1028, 1032, 1037,
		1042, 1047, 1052, 1057, 1062, 1067, 1071, 1076, 1081, 1086,
		1091, 1096, 1101, 1106, 1111, 1115, 1120, 1125, 1130, 1135,
		1140, 1145, 1150, 1154, 1159, 1164, 1169, 1174, 1179, 1184,
		1189, 1194, 1198, 1203, 1208, 1213, 1218, 1223, 1228, 1233,
		1237, 1242, 1247, 1252, 1257, 1262, 1267, 1272, 1277, 1281,
		1286, 1291, 1296, 1301, 1306, 1311};

volatile uint16_t thftable[256] = {1311, 1326, 1341, 1356, 1371, 1387, 1402, 1417, 1432, 1447,
		1462, 1478, 1493, 1508, 1523, 1538, 1553, 1568, 1584, 1599,
		1614, 1629, 1644, 1659, 1675, 1690, 1705, 1720, 1735, 1750,
		1766, 1781, 1796, 1811, 1826, 1841, 1857, 1872, 1887, 1902,
		1917, 1932, 1948, 1963, 1978, 1993, 2008, 2023, 2039, 2054,
		2069, 2084, 2099, 2114, 2130, 2145, 2160, 2175, 2190, 2205,
		2221, 2236, 2251, 2266, 2281, 2296, 2311, 2327, 2342, 2357,
		2372, 2387, 2402, 2418, 2433, 2448, 2463, 2478, 2493, 2509,
		2524, 2539, 2554, 2569, 2584, 2600, 2615, 2630, 2645, 2660,
		2675, 2691, 2706, 2721, 2736, 2751, 2766, 2782, 2797, 2812,
		2827, 2842, 2857, 2873, 2888, 2903, 2918, 2933, 2948, 2964,
		2979, 2994, 3009, 3024, 3039, 3054, 3070, 3085, 3100, 3115,
		3130, 3145, 3161, 3176, 3191, 3206, 3221, 3236, 3252, 3267,
		3282, 3297, 3312, 3327, 3343, 3358, 3373, 3388, 3403, 3418,
		3434, 3449, 3464, 3479, 3494, 3509, 3525, 3540, 3555, 3570,
		3585, 3600, 3616, 3631, 3646, 3661, 3676, 3691, 3707, 3722,
		3737, 3752, 3767, 3782, 3797, 3813, 3828, 3843, 3858, 3873,
		3888, 3904, 3919, 3934, 3949, 3964, 3979, 3995, 4010, 4025,
		4040, 4055, 4070, 4086, 4101, 4116, 4131, 4146, 4161, 4177,
		4192, 4207, 4222, 4237, 4252, 4268, 4283, 4298, 4313, 4328,
		4343, 4359, 4374, 4389, 4404, 4419, 4434, 4450, 4465, 4480,
		4495, 4510, 4525, 4540, 4556, 4571, 4586, 4601, 4616, 4631,
		4647, 4662, 4677, 4692, 4707, 4722, 4738, 4753, 4768, 4783,
		4798, 4813, 4829, 4844, 4859, 4874, 4889, 4904, 4920, 4935,
		4950, 4965, 4980, 4995, 5011, 5026, 5041, 5056, 5071, 5086,
		5102, 5117, 5132, 5147, 5162, 5177};

// Debounce
	uint16_t get_key_press( uint16_t key_mask );
	uint16_t get_key_rpt( uint16_t key_mask );
	uint16_t get_key_state( uint16_t key_mask );
	uint16_t get_key_short( uint16_t key_mask );
	uint16_t get_key_long( uint16_t key_mask );
	volatile uint16_t key_state;                                // debounced and inverted key state:                                                  // bit = 1: key pressed
	volatile uint16_t key_press;                                // key press detect
	volatile uint16_t key_rpt;
	volatile uint16_t button_in;		// map input buttons
	volatile uint16_t buttons_pressed;  // buttons debounced
	//Programmer
	uint8_t parameter_set;
	uint8_t parameter_set_old;
	uint8_t parameter_leds;
	uint8_t parameter_leds_old;
	uint8_t parameter_secondpage;
	uint8_t parameter_secondpageold;
	uint8_t testcount;
	// LFO
	volatile float global_sample_freq = 5000;
	volatile float global_sample_freq_dac = 5000;
	volatile uint8_t countdirection[2];
	volatile uint32_t lfocountervaluecalc[2];
	volatile uint32_t lfocountervalue[2];
	volatile uint8_t lfo_timer_count;
	volatile uint8_t lfoparamtervalue[2];
	volatile uint32_t lfovalue[2];
	volatile uint32_t lfovaluerect[2];
	volatile uint32_t lfospeed[2];
	volatile uint8_t lfowaveform;
	volatile uint16_t lfo_output[2];
	volatile uint16_t lfo_multiplicator=8;
	volatile uint16_t lfo_base_freq = 0.25;
	volatile uint8_t lfo_render_flag;
	volatile uint32_t lfo_max_counter_value;
	volatile uint32_t lfo_dac_value;
	volatile uint8_t lfo_audio;
	uint8_t lfo_range = 1;
	uint8_t lfo_range_old;
	//Phase based Oscillator
	volatile const double mPI = 3.14159265359;
	volatile const double twoPI = 6.283185307;
	volatile double mFrequency;
	volatile uint32_t mPhase = 0;
	volatile uint32_t mSampleRate = 50000;
    volatile uint32_t mPhaseIncrement = 2000;
    volatile uint32_t mPhaseIncrement_int = 60;
    uint32_t mIncrement_counter = 0;
    volatile uint16_t getbuffer;
    uint32_t getphase;
    volatile double increment_calc_help = 1;
    uint32_t dac_watch;
    uint32_t render_watch;
    uint32_t dr_watch;
	// Envelope
	volatile float adsr_parametervalue;
	volatile uint32_t adsr_countervalue;
	volatile uint32_t adsr_value;
	volatile uint16_t adsr_output;
	volatile float adsr_attack;
	volatile uint32_t adsr_attackcountervalue;
	volatile uint8_t adsr_attackcalc;
	volatile float adsr_decay;
	volatile uint32_t adsr_decaycountervalue;
	volatile uint8_t adsr_decaycalc;
	volatile float adsr_release;
	volatile uint32_t adsr_releasecountervalue;
	volatile uint8_t adsr_releasecalc;
	volatile float adsr_sustain;
	volatile uint8_t adsr_sustain_int;
	volatile uint8_t envelope_running;
	volatile uint8_t adsr_stage;
	volatile float adsr_calc1;
	volatile float adsr_calc2;
	volatile float adsr_outputcalc;
	volatile float adsr_transfercalc2;
	volatile float adsr_transfercalc1;
	volatile uint32_t adsr_scalc1;
	volatile uint32_t adsr_scalc2;
	volatile uint8_t ad_ar_select=1;
	volatile uint8_t loop = 0;
	volatile uint8_t timer4count=0;
	volatile uint8_t adsr_trigger_count;
	//DAC
	volatile uint16_t dac_out1;
	volatile uint16_t dac_out2;
	volatile uint8_t dac_select;
	volatile uint16_t dac_getvalue1;
	volatile uint16_t dac_getvalue2;
	volatile uint16_t dac_extern_getvalue1; // Resonance
	volatile uint16_t dac_extern_getvalue2;
	volatile float dac_calc1;
	volatile float dac_calc2;
	volatile float dac_calc3;
	volatile uint16_t buffer[256];
	volatile uint32_t dac_buffer_count = 0;
	volatile uint32_t render_buffer_count = 0;
	volatile uint32_t size_of_buffer;
	volatile float nIncrement;
	// ADC
	volatile uint8_t ai_speed;
	volatile uint8_t ai_speed_old;
	volatile uint8_t ai_attack;
	volatile uint8_t ai_decay;
	volatile uint8_t ai_extern;
	volatile uint8_t adc_flag_count;
	volatile uint8_t adc_channel=0;
	volatile uint16_t adc_value[6];
	volatile uint16_t adc_value_sum[6];
	volatile uint8_t adc_averagecount;
	volatile uint16_t adc_old;
	volatile uint8_t adc_flag = 1;
	volatile uint8_t adc_flag_2 = 0;
	volatile uint16_t countx;
	volatile uint8_t channelnumbers[4] = {0,1,2,3};
	volatile uint8_t adc_unlocknow;
	//MIDI
	volatile uint8_t notetableposition[2] = {16, 16};
	volatile uint8_t midibuffer[4];
	volatile uint8_t midibytes;
	volatile uint8_t midinote = 20;
	volatile uint8_t midicompleteflag;
	volatile uint8_t midistatus;
	volatile uint8_t midichannelrx;
	volatile uint8_t midimodwheel;
	volatile uint8_t midistat;
	volatile uint8_t miditestbyte1 = 0b10010101;
	volatile uint8_t miditestbyte2 = 0;
	volatile uint8_t midiupdate = 0;
	volatile uint8_t parameter_value[12][20];
	//SPI
	volatile uint8_t spi_send_flag;
	volatile uint16_t dummy_value;
	//Trigger
	volatile uint8_t global_trigger = 0;
	volatile uint8_t global_gate = 0;
	volatile uint8_t global_gate_old = 0;
	//Heartbeat
	volatile uint16_t main_cycles;

int main(void)
{

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
//  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start(&htim7);

//	ADC1->CR1 |= (1<<5)|(1<<8)|(1<<11); //EOC, Scanmode,  Discontinuous mode on regular channels
//	ADC1->CR2 |=(1<<20); //EXTTRIG: External trigger conversion mod
//	ADC1->SQR3 = 1;

	//Switch Voltage Regulator on
//	ADC1->CR |= (1<<28);
//	HAL_Delay(500);
//	//Switch on ADC
//	ADC1->CR |= (1<<0);

	//Calibration
	ADC1->CR &= ~(1<<0);
	ADC1->CFGR1 &= ~(1<<0);
	ADC1->CR |= (1<<31);
	while(ADC1->CR &(1<<31));

	//Switch on ADC
	ADC1->CR |= (1<<0);
	ADC1->SMPR = 5 ;
	ADC1->CFGR1 |= (1<<21);
	ADC1->CFGR1 &= ~(1<<2);
	ADC1->IER |= (1<<3);
	//Set Channel
	ADC1->CHSELR = 0;
	//Cont
	ADC1->CFGR1 &= ~(1<<13);
	//ADC Start
	ADC1->CR |= (1<<2);


	SPI2->CR2 |=(1<<6);
	SPI2->CR2 &= ~(1<<3);

	SPI2->CR1 |=(1<<6);


	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin( SPI2_SHDN_GPIO_Port, SPI2_SHDN_Pin, GPIO_PIN_SET);  //Shutdonw
	HAL_GPIO_WritePin(SPI2_LATCH_GPIO_Port, SPI2_LATCH_Pin, GPIO_PIN_RESET);				  //LATCH
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);



	ADC1->CR |=  (1<<0);
	HAL_Delay(500);
	mPhaseIncrement = tfftable[10];
	render_lfo();
	print_leds();

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);




  while (1)
  {
		heart_beat();
		render_lfo();
		process_buttons();
		calc_data();

		if(adc_flag_2){
			adc_flag_2 = 0 ;
			adc_start();
		}

		if(ai_speed != ai_speed_old || lfo_range != lfo_range_old){
			ai_speed_old = ai_speed;
			lfo_range_old = lfo_range;
			switch (lfo_range){
			case 0: mPhaseIncrement = thftable[ai_speed]; break;
			case 1: mPhaseIncrement = tfftable[ai_speed]; break;
			case 2: mPhaseIncrement = tlftable[ai_speed]; break;
			default: break;
			}

		}
  }//end infinity loop
  /* USER CODE END 3 */
}//end main



void heart_beat(void){
	main_cycles++;
	if(main_cycles == 500){
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		main_cycles = 0;

	}
}

void print_leds(void){
	if(parameter_secondpage==0){
		if(parameter_set &(1<<0))
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
		else
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);

		if(parameter_set &(1<<1))
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
		else
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);

		if(parameter_set &(1<<2))
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
		else
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);

		if(parameter_set &(1<<3))
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, SET);
		else
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, RESET);
	}
	else{
		if(parameter_set &(1<<4))
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
		else
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);

		if(parameter_set &(1<<5))
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
		else
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);

		if(parameter_set &(1<<6))
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
		else
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);

		if(parameter_set &(1<<7))
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, SET);
		else
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, RESET);
	}

	if(parameter_secondpage){
		HAL_GPIO_WritePin(LED_Page_GPIO_Port, LED_Page_Pin, SET);
	}
	else{
		HAL_GPIO_WritePin(LED_Page_GPIO_Port, LED_Page_Pin, RESET);
	}

}

void process_buttons(void){
	if(HAL_GPIO_ReadPin(ADSR_GPIO_Port, ADSR_Pin))
			button_in |= (1<<0);
		else
			button_in &= ~(1<<0);

		if(HAL_GPIO_ReadPin(LFO_WAVE_1_GPIO_Port, LFO_WAVE_1_Pin))
			button_in |= (1<<1);
		else
			button_in &= ~(1<<1);

		if(HAL_GPIO_ReadPin(LFO_WAVE_2_GPIO_Port, LFO_WAVE_2_Pin))
			button_in |= (1<<2);
		else
			button_in &= ~(1<<2);

//		if(HAL_GPIO_ReadPin(SP_GPIO_Port, SP_Pin ))
//			button_in |= (1<<3);
//		else
//			button_in &= ~(1<<3);

		if(HAL_GPIO_ReadPin(PUSH1_GPIO_Port, PUSH1_Pin ))
			button_in &= ~(1<<4);
		else
			button_in |= (1<<4);

		if(HAL_GPIO_ReadPin(PUSH2_GPIO_Port, PUSH2_Pin ))
			button_in &= ~(1<<5);
		else
			button_in |= (1<<5);

		if(HAL_GPIO_ReadPin(PUSH3_GPIO_Port, PUSH3_Pin ))
			button_in &= ~(1<<6);
		else
			button_in |= (1<<6);

		if(HAL_GPIO_ReadPin(PUSH4_GPIO_Port, PUSH4_Pin ))
			button_in &= ~(1<<7);
		else
			button_in |= (1<<7);

		if(HAL_GPIO_ReadPin(TRIGGER_GPIO_Port, TRIGGER_Pin )){
			button_in &= ~(1<<8);
			global_gate=0;
		}
		else{
			button_in |= (1<<8);
			global_gate=1;

		}
		//Debounce
		buttons_pressed = get_key_press(button_in);

		// Second Page
		if(HAL_GPIO_ReadPin(SP_GPIO_Port, SP_Pin)){
			HAL_GPIO_WritePin(LED_Page_GPIO_Port, LED_Page_Pin, SET);
			parameter_secondpage = 1;
		}
		else{
			HAL_GPIO_WritePin(LED_Page_GPIO_Port, LED_Page_Pin, RESET);
			parameter_secondpage = 0;
		}


		// Pushbuttons 1 - Loop AD/R
		if(buttons_pressed &(1<<4)){
			if(parameter_secondpage){
				parameter_set ^= (1<<4);
				parameter_leds ^= (1<<4);
				}
			else{
				parameter_set ^= (1<<0);
				parameter_leds ^= (1<<0);
				}
		}

		// Pushbuttons 2 - LFO Audio
		if(buttons_pressed &(1<<5)){
			if(parameter_secondpage){
				parameter_set ^= (1<<5);
				parameter_leds ^= (1<<5);
				}
			else{
				parameter_set ^= (1<<1);
				parameter_leds ^= (1<<1);
				lfo_audio^= 1;
				}

		}
		// Pushbuttons 3 - ENV Loop
		if(buttons_pressed &(1<<6)){
			if(parameter_secondpage){
				parameter_set ^= (1<<6);
				parameter_leds ^= (1<<6);
				testcount++;
				}
			else{
				parameter_set ^= (1<<2);
				parameter_leds ^= (1<<2);
				loop ^= 1;
				testcount++;
				}
		}

		// Pushbuttons 4 - ENV Kurve
		if(buttons_pressed &(1<<7)){
			if(parameter_secondpage){
				parameter_set ^= (1<<7);
				parameter_leds ^= (1<<7);
				}
			else{
				parameter_set ^= (1<<3);
				parameter_leds ^= (1<<3);
				}
		}

		if(parameter_set != parameter_set_old){
			parameter_set_old = parameter_set;
			print_leds();
		}

		if(parameter_secondpage != parameter_secondpageold){
			 parameter_secondpageold = parameter_secondpage;
			 print_leds();
		}


//		loop ^= 1;
//		lfo_audio^= 1;


		// Select AD / AR
		if(HAL_GPIO_ReadPin(ADSR_GPIO_Port, ADSR_Pin)){
			ad_ar_select = 1;

		}
		else{
			ad_ar_select = 0;

		}

		//LFO Wave (Triangle, Square, Saw)
		if(HAL_GPIO_ReadPin(LFO_WAVE_1_GPIO_Port, LFO_WAVE_1_Pin )){
			lfowaveform |= (1<<0);
		}
		else{
			lfowaveform &= ~(1<<0);
		}

		if(HAL_GPIO_ReadPin(LFO_WAVE_2_GPIO_Port, LFO_WAVE_2_Pin)){
			lfowaveform |= (1<<1);
		}
		else{
			lfowaveform &= ~(1<<1);
		}

		//LFO Range (Low, Full, High)
		if(HAL_GPIO_ReadPin(LFO_RANGE_1_GPIO_Port, LFO_RANGE_1_Pin )){
			lfo_range |= (1<<0);
		}
		else{
			lfo_range &= ~(1<<0);
		}

		if(HAL_GPIO_ReadPin(LFO_RANGE_2_GPIO_Port, LFO_RANGE_2_Pin)){
			lfo_range |= (1<<1);
		}
		else{
			lfo_range &= ~(1<<1);
		}
}

void render_lfo(void){
	//LFO 1
	uint16_t i = 0;
	uint16_t k = 0;

	if(render_buffer_count <= dac_buffer_count){
		k = dac_buffer_count;
	}
	else{
		k = 256;
	}

	for (i = render_buffer_count; i<k; i++){
		render_buffer_count++;
		if( render_buffer_count == 256){
			render_watch++;
			render_buffer_count = 0;
		}
		mPhase = mPhase + mPhaseIncrement;
		//Phase Overflow
		if(mPhase >= 65536){
			mPhase = mPhase - 65536;
		}

		getphase = mPhase; // Concert double to int
		//getphase &= ~(0b0000000000000000); //Modulo 65536, mask integer bits
		getbuffer = (getphase>>8);
		switch (lfowaveform){
		case 0: buffer[i] = (getphase>>4); break;
		case 1: if (getphase<32763){
					buffer[i] = 4095;
				}
				else{
					buffer[i] = 0;
				}
				break;
		case 2: if (getphase<65526){
					buffer[i] = 2*getphase;
				}
				else{
					buffer[i] = 4095-(2*getphase);
				}
				break;
		default: break;
		}
		//sinetable[getbuffer];

		}
}

void calc_data(void){

	  // Envelope calc data
	  adsr_attackcalc = ~ai_attack;
//	  adsr_attack = 2*log_table[ai_attack];
	  adsr_attack= (adsr_attack/256) + 0.5;
	  adsr_attackcountervalue = (4096<<14)/(5000/adsr_attack);

	  adsr_decaycalc = ~ai_decay;
//	  adsr_decay =  8*log_table[ai_decay];
	  adsr_decay =  (adsr_decay/256) + 0.5;
	  adsr_decaycountervalue = (4096<<14)/(5000/adsr_decay);

	  adsr_releasecalc =~ai_decay;
	  adsr_release = 10*adsr_releasecalc;
	  adsr_release = adsr_release/256 + 0.125;
	  adsr_releasecountervalue = (4096<<14)/(5000/adsr_release);

	  adsr_sustain = 255;


}

void SPI2_IRQHandler(void)
{
//  /* USER CODE BEGIN SPI2_IRQn 0 */
	if(spi_send_flag){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, SET);
		dummy_value = SPI2 -> DR;
		spi_send_flag=0;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, RESET);
		SPI2 -> DR = dac_extern_getvalue2;
	}
	else{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, SET);
		dummy_value = SPI2 -> DR;
//		SPI2->CR2 &= ~(1<<6);
	}

  HAL_SPI_IRQHandler(&hspi2);

}


// Audiosampler and DAC Out
void TIM7_IRQHandler(void){
//	dummy_value = SPI2 -> DR;
	dac_extern_getvalue1 = 0b0011000000000001  |buffer[dac_buffer_count];
	dac_extern_getvalue2 = 0b1011000000000000  |(adsr_output);
	spi_send_flag =1;
	SPI2->CR1 |=(1<<6);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, RESET);
	SPI2 -> DR = dac_extern_getvalue1;
	lfo_output[0] = buffer[dac_buffer_count];
	if(lfo_audio){
	dac_buffer_count++;
	}
	else{
		lfo_timer_count++;
		if(lfo_timer_count == 50){
			lfo_timer_count = 0;
			dac_buffer_count++;
		}

	}
	if (dac_buffer_count == 256){
		dac_watch++;
		dac_buffer_count = 0;
	}
	//ADSR
	if(global_trigger){
		adsr_stage = 1;
		adsr_value = 0;
		global_trigger = 0;
	}

	adsr_trigger_count++;
	if(adsr_trigger_count == 10){
		adsr_trigger_count = 0;
		if(ad_ar_select){
			switch(adsr_stage){
			case 0: break;
			case 1: adsr_value= adsr_value + adsr_attackcountervalue;
					if(adsr_value>4096<<14){
						adsr_value = 4095<<14; //255<<18
						adsr_stage = 2;
					}
					adsr_output = adsr_value>>14;
					break;
			case 2: adsr_value= adsr_value - adsr_decaycountervalue;
					adsr_output = (adsr_value>>14);
					if(adsr_value>1000000000){
						adsr_value = 0;
						adsr_output = (adsr_value>>14);
						if(loop){
						adsr_stage = 1;
						}
						else{
						adsr_stage = 0;
						}
					break;
			default: break;
			}
			}
		}
			else{
				switch(adsr_stage){
				case 0: break;
				case 1: adsr_value= adsr_value + adsr_attackcountervalue;
						if(adsr_value>4096<<14){
							adsr_value = 4095<<14; //255<<18
							adsr_stage = 2;
						}
						adsr_output = adsr_value>>14;
						break;
				case 2:	adsr_output = adsr_sustain;
						if(global_gate == 0){
							adsr_stage = 3;
							adsr_sustain_int = adsr_sustain;
							adsr_value =  4095<<14;
						}
						adsr_output = adsr_value>>14;
						break;
				case 3: adsr_value = adsr_value - adsr_releasecountervalue;
						if(adsr_value>1000000000){
							adsr_value = 0;
							if(loop){
							adsr_stage = 1;
							}
							else{
							adsr_stage = 0;
							}
						}
						adsr_output = (adsr_value>>14);
						break;
				default: break;
			}
			}
	}
	    HAL_TIM_IRQHandler(&htim7);
}
//void adc_avg_filter(void){
//	volatile uint16_t adc_value_sum[6];
//	volatile uint8_t adc_averagecount;
//}

void TIM3_IRQHandler(void)
{
adc_flag_2 = 1 ;
adc_flag_count++;

  HAL_TIM_IRQHandler(&htim3);

}

void ADC1_IRQHandler(void)
{
	// Conversion done

	adc_flag = 1;
//	HAL_ADC_Stop_IT (&hadc1);
	ADC1->ISR &= ~(1<<5);
//	HAL_GPIO_TogglePin(Trigger_Output_GPIO_Port, Trigger_Output_Pin);
	HAL_ADC_IRQHandler(&hadc1);
}

void adc_start(void){
	if(adc_flag){
		// Add ADC result to sum variable
		switch(adc_channel){
		case 0:ai_speed = (ADC1->DR>>4);break;
		case 1:ai_attack = (ADC1->DR>>4);break; //adc_value_sum[1] + adc_value_sum[0] +
		case 2:ai_decay = (ADC1->DR>>4);break; //adc_value_sum[1] +
		case 3:ai_extern = (ADC1->DR>>4);break; //adc_value_sum[3]
		default: break;
		}
//		adc_averagecount++;
//		if (adc_averagecount == 64){
//			adc_averagecount = 0;
//			ai_speed = adc_value_sum[0]>>4;
//			ai_attack = adc_value_sum[1]>>4;
//			ai_decay = adc_value_sum[2]>>4;
//			ai_extern = adc_value_sum[3]>>4;
//			for(int i = 0; i<4; i++){
//			adc_value_sum[i]=0;
//			}
//		}


		// Switch ADC channel
		adc_channel++;
			if(adc_channel == 4){
				adc_channel = 0;
			}
		// Switch ADC channel ADC register
		ADC1->CHSELR = channelnumbers[adc_channel];
		adc_flag = 0;

		ADC1->IER |= (1<<3); // Start Interrupt
		ADC1->CR |= (1<<2);	//Start Conversion

		}
}

void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	global_trigger = 1;
	testcount++;
  /* USER CODE END EXTI0_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(Trigger_In_Pin);
	 HAL_GPIO_EXTI_IRQHandler(TRIGGER_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if ( GPIO_Pin == TRIGGER_Pin)
    {	testcount++;
    	global_trigger = 1;
//    	HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
    }
}



// Button debouncing

///////////////////////////////////////////////////////////////////
//
// check if a key has been pressed. Each pressed key is reported
// only once
//
uint16_t get_key_press( uint16_t key_mask )
{
	HAL_TIM_Base_Stop(&htim6);                                          // read and clear atomic !
	key_mask &= key_press;                          // read key(s)
	key_press ^= key_mask;                          // clear key(s)
	HAL_TIM_Base_Start(&htim6);
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
// check if a key has been pressed long enough such that the
// key repeat functionality kicks in. After a small setup delay
// the key is reported being pressed in subsequent calls
// to this function. This simulates the user repeatedly
// pressing and releasing the key.
//
uint16_t get_key_rpt( uint16_t key_mask )
{
	HAL_TIM_Base_Stop(&htim6);                                          // read and clear atomic !
	key_mask &= key_rpt;                            // read key(s)
	key_rpt ^= key_mask;                            // clear key(s)
	HAL_TIM_Base_Start(&htim6);
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
// check if a key is pressed right now
//
uint16_t get_key_state( uint16_t key_mask )

{
	key_mask &= key_state;
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
uint16_t get_key_short( uint16_t key_mask )
{
	HAL_TIM_Base_Stop(&htim6);                                          // read key state and key press atomic !
	return get_key_press( ~key_state & key_mask );
}

///////////////////////////////////////////////////////////////////
//
uint16_t get_key_long( uint16_t key_mask )
{
	return get_key_press( get_key_rpt( key_mask ));
}

void TIM6_IRQHandler (void){

	static uint16_t ct0 = 0xFF, ct1 = 0xFF,  rpt;
	uint16_t i;

	i = key_state ^ ~KEY_PIN;                       // key changed ?
	ct0 = ~( ct0 & i );                             // reset or count ct0
	ct1 = ct0 ^ (ct1 & i);                          // reset or count ct1
	i &= ct0 & ct1;                                 // count until roll over ?
	key_state ^= i;                                 // then toggle debounced state
	key_press |= key_state & i;                     // 0->1: key press detect

	if( (key_state & REPEAT_MASK) == 0 )            // check repeat function
		rpt = REPEAT_START;                          // start delay
	if( --rpt == 0 ){
		rpt = REPEAT_NEXT;                            // repeat delay
		key_rpt |= key_state & REPEAT_MASK;
	}
	HAL_TIM_IRQHandler(&htim6);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

// Debounce
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1280-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 640-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1280-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_USART1_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART1_Init 0 */
//
//  /* USER CODE END USART1_Init 0 */
//
//  /* USER CODE BEGIN USART1_Init 1 */
//
//  /* USER CODE END USART1_Init 1 */
//  huart1.Instance = USART1;
//  huart1.Init.BaudRate = 115200;
//  huart1.Init.WordLength = UART_WORDLENGTH_8B;
//  huart1.Init.StopBits = UART_STOPBITS_1;
//  huart1.Init.Parity = UART_PARITY_NONE;
//  huart1.Init.Mode = UART_MODE_TX_RX;
//  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
//  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART1_Init 2 */
//
//  /* USER CODE END USART1_Init 2 */
//
//}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Page_GPIO_Port, LED_Page_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_NSS_Pin|SPI2_LATCH_Pin|SPI2_SHDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LinLog_Pin */
  GPIO_InitStruct.Pin = SP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Gate_Pin */
  GPIO_InitStruct.Pin = LED_Page_Pin | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Page_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PUSH1_Pin PUSH2_Pin PUSH3_Pin PUSH4_Pin
                           LFO_WAVE_1_Pin LFO_WAVE_2_Pin LFO_RANGE_1_Pin LFO_RANGE_2_Pin
                           ADSR_Pin TRIGGER_Pin */
  GPIO_InitStruct.Pin = PUSH1_Pin|PUSH2_Pin|PUSH3_Pin|PUSH4_Pin
                          |LFO_WAVE_1_Pin|LFO_WAVE_2_Pin|LFO_RANGE_1_Pin|LFO_RANGE_2_Pin
                          |ADSR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_NSS_Pin SPI2_LATCH_Pin SPI2_SHDN_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS_Pin|SPI2_LATCH_Pin|SPI2_SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  //EXTI9
  GPIO_InitStruct.Pin = TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
