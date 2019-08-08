#include "common.h"
#include "include.h"
#include "stdio.h"
#include "stdbool.h"
#include "math.h"


#define	Motor_Power_Init	gpio_init(PTB17, GPO, 1)
#define	Motor_Power_On		gpio_set(PTB17, 0)
#define	Motor_Power_Off		gpio_set(PTB17, 1)

void PIT0_IRQHandler(void);

void LED_init(void)
{
	gpio_init(PTC17, GPO, 0);	//D1
	gpio_init(PTC16, GPO, 0);	//D0
	gpio_init(PTC19, GPO, 0);	//DC
	gpio_init(PTC18, GPO, 1);	//RST    
	LCD_Init();
}

//ADC通道状态检测
uint32 ADC16_GetChannelStatusFlags(ADCn_e base, uint32 channelGroup)
{
	ADC_MemMapPtr adcn[2] = { ADC0_BASE_PTR, ADC1_BASE_PTR };
	uint32 ret = 0U;

	if (0U != (adcn[base]->SC1[channelGroup] & ADC_SC1_COCO_MASK))
	{
		ret |= ADC_SC1_COCO_MASK;
	}
	return ret;
}

//ADC获取状态标志位
uint32 ADC16_GetStatusFlags(ADCn_e base)
{
	ADC_MemMapPtr adcn[2] = { ADC0_BASE_PTR, ADC1_BASE_PTR };
	uint32 ret = 0;

	if (0U != (adcn[base]->SC2 & ADC_SC2_ADACT_MASK))
	{
		ret |= ADC_SC2_ADACT_MASK;
	}
	return ret;
}

//ADC自矫正
bool ADC16_DoAutoCalibration(ADCn_e base)
{
	ADC_MemMapPtr adcn[2] = { ADC0_BASE_PTR, ADC1_BASE_PTR };
	bool bHWTrigger = false;
	volatile uint32 tmp32;
	bool status = true;

	if (0U != (ADC_SC2_ADTRG_MASK & adcn[base]->SC2))
	{
		bHWTrigger = true;
		adcn[base]->SC2 &= ~ADC_SC2_ADTRG_MASK;
	}

	adcn[base]->SC3 |= ADC_SC3_CAL_MASK | ADC_SC3_CALF_MASK;
	while (0U == (ADC_SC1_COCO_MASK & ADC16_GetChannelStatusFlags(base, 0U)))
	{
		if (0U != (ADC_SC1_COCO_MASK & ADC16_GetStatusFlags(base)))
		{
			status = false;
			break;
		}
	}
	tmp32 = adcn[base]->R[0];

	if (bHWTrigger)
	{
		adcn[base]->SC2 |= ADC_SC2_ADTRG_MASK;
	}

	if (0U != (ADC_SC3_CALF_MASK & ADC16_GetStatusFlags(base)))
	{
		status = false;
	}
	if (true != status)
	{
		return status;
	}

	tmp32 = adcn[base]->CLP0 + adcn[base]->CLP1 + adcn[base]->CLP2 + adcn[base]->CLP3 + adcn[base]->CLP4 + adcn[base]->CLPS;
	tmp32 = 0x8000U | (tmp32 >> 1U);
	adcn[base]->PG = tmp32;

	return status;
}

//ADC硬件均值滤波设定
typedef enum _adc16_hardware_average_mode
{
	kADC16_HardwareAverageCount4 = 0U,   /*!< For hardware average with 4 samples. */
	kADC16_HardwareAverageCount8 = 1U,   /*!< For hardware average with 8 samples. */
	kADC16_HardwareAverageCount16 = 2U,  /*!< For hardware average with 16 samples. */
	kADC16_HardwareAverageCount32 = 3U,  /*!< For hardware average with 32 samples. */
	kADC16_HardwareAverageDisabled = 4U, /*!< Disable the hardware average feature.*/
} adc16_hardware_average_mode_t;

//ADC设置硬件均值滤波等级
void ADC16_SetHardwareAverage(ADCn_e base, adc16_hardware_average_mode_t mode)
{
	ADC_MemMapPtr adcn[2] = { ADC0_BASE_PTR, ADC1_BASE_PTR };
	uint32 tmp32 = adcn[base]->SC3 & ~(ADC_SC3_AVGE_MASK | ADC_SC3_AVGS_MASK);

	if (kADC16_HardwareAverageDisabled != mode)
	{
		tmp32 |= ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(mode);
	}
	adcn[base]->SC3 = tmp32;
}

//充电相关宏定义与变量
#define MPU_Voltage		3.330
#define DAC_SCALE		1 / (MPU_Voltage / 4096)
#define CCCV_DAC		DAC0
#define CCCV_ADC_MOD	ADC1
#define CCCV_ADC		ADC1_SE13	//PTB7
#define CCCV_GET_BAT	adc_once(CCCV_ADC,ADC_16bit)
#define CCCV_BAT_MAG	6	
#define CCCV_ADC_SCALE	CCCV_BAT_MAG * MPU_Voltage / 65536
#define CCCV_RES		0.02222  //0.025
#define CCCV_CAl_MAG	CCCV_RES * 30
#define CCCV_SWITCH     PTB16
#define CCCV_ENABLE		gpio_set(CCCV_SWITCH, 1)
#define CCCV_DISABLE	gpio_set(CCCV_SWITCH, 0)
typedef enum CCCV_STATUS
{
	CCCV_Perparing = 0,
	CCCV_Charging,
	CCCV_Finished,
} CCCV_STATUS;
CCCV_STATUS cccv_status = CCCV_Perparing;
float cccv_power = 0;
float cccv_battery = 0;
//充电电压
#define CCCV_VOL		12
//充电功率
#define CCCV_POWER		20 //26
//充电前延时
#define CCCV_DELAY		5000

//DAC电压输出
void dac_voltage(DACn_e dac, float vol)
{
	uint16 op;
	if (vol >= MPU_Voltage)vol = MPU_Voltage;
	op = (uint16)(vol * DAC_SCALE);
	if (op >= 4095) op = 4095;
	dac_out(dac, op);
}

//充电初始化
void cccv_init()
{
	gpio_init(CCCV_SWITCH, GPO, 0);
	dac_init(CCCV_DAC);
	adc_init(CCCV_ADC);
	ADC16_SetHardwareAverage(CCCV_ADC_MOD, kADC16_HardwareAverageCount32);
	while(!ADC16_DoAutoCalibration(CCCV_ADC_MOD));
}

//充电反馈输出
void cccv_output(float power)
{
	float output;
	cccv_battery = CCCV_ADC_SCALE * CCCV_GET_BAT;
	if (cccv_battery <= 0)cccv_battery = 0.1;
	output = CCCV_CAl_MAG * power / cccv_battery;
	dac_voltage(CCCV_DAC, output);
}


uint8 cmd[5] = 0;
uint8 data[5] = 0;
int16 acc = 0;

// 主函数
void  main(void)
{
	char text[15];	//LCD字符串缓存

	Motor_Power_Init;
	DisableInterrupts;
	LED_init();

	FTM_COMP_init(FTM1, FTM_G0, 10000, 1000, 500);
	//FTM_PWM_init(FTM1, FTM_CH0, 10000, 1000);
	//cccv_init();
	//pit_init_ms(PIT0, 20);
	//set_vector_handler(PIT0_VECTORn, PIT0_IRQHandler);
	//enable_irq(PIT0_IRQn);
	EnableInterrupts;

	//spi_init(SPI0, SPIn_PCS1, MASTER, 10000);
	//cmd[0] = 0x20; cmd[1] = 0x0F;
	//spi_mosi(SPI0, SPIn_PCS1, cmd, NULL, 2);
	//cmd[0] = 0x21; cmd[1] = 0x00;
	//spi_mosi(SPI0, SPIn_PCS1, cmd, NULL, 2);	
	//cmd[0] = 0x22; cmd[1] = 0x08;
	//spi_mosi(SPI0, SPIn_PCS1, cmd, NULL, 2);	
	//cmd[0] = 0x23; cmd[1] = 0x30;
	//spi_mosi(SPI0, SPIn_PCS1, cmd, NULL, 2);	
	//cmd[0] = 0x24; cmd[1] = 0x00;
	//spi_mosi(SPI0, SPIn_PCS1, cmd, NULL, 2);

	while (1)
	{
		LCD_P6x8Str(0, 1, "Complementary");
		//////WHO_AM_I
		////cmd[0] = 0x8F; cmd[1] = 0xFF;
		////spi_mosi(SPI0, SPIn_PCS1, cmd, data, 2);
		//cmd[0] = OUT_X_H | 0x80; cmd[1] = 0xFF;
		//spi_mosi(SPI0, SPIn_PCS1, cmd, data, 2);
		//acc = data[1] << 8;
		//cmd[0] = OUT_X_L | 0x80; cmd[1] = 0xFF;
		//spi_mosi(SPI0, SPIn_PCS1, cmd, data, 2);
		//acc += data[1];
		//sprintf(text, "%d          ", (int)acc);
		//LCD_P6x8Str(0, 2, text);
		////if	(cccv_status == CCCV_Perparing)LCD_P6x8Str(0, 1, "Perparing   ");
		////else if (cccv_status == CCCV_Charging)LCD_P6x8Str(0, 1, "Charging    ");
		////else if (cccv_status == CCCV_Finished)LCD_P6x8Str(0, 1, "Finished    ");
		//////sprintf(text, "BAT %d mV          ", (int)(CCCV_GET_BAT * CCCV_ADC_SCALE * 1000));
		////sprintf(text, "BAT %d mV          ", (int)(cccv_battery * 1000));
		////LCD_P6x8Str(0, 2, text);
		////sprintf(text, "POW %d mW          ", (int)(cccv_power * 1000));
		////LCD_P6x8Str(0, 3, text);
		DELAY_MS(500);
	}
}

void PIT0_IRQHandler()
{
	PIT_Flag_Clear(PIT0); 
	static uint32 timer = 0;
	timer += 20;
	if ((cccv_status == CCCV_Perparing) && (timer >= CCCV_DELAY))
	{
		cccv_status = CCCV_Charging;
		CCCV_ENABLE;
		timer = 0;
	}
	if (cccv_status == CCCV_Charging)
	{
		//cccv_power = timer / 10000.0 * CCCV_POWER;
		//cccv_power = 1.5 * log(timer / 1000.0) + 15.6;
		//cccv_power = 2.6 * log(timer / 1000.0) + 13;
		cccv_power = 2.6 * log(timer / 1000.0) + 12;
		if (cccv_power >= CCCV_POWER) cccv_power = CCCV_POWER;
		cccv_output(cccv_power);
		if (cccv_battery >= CCCV_VOL)
		{
			cccv_status = CCCV_Finished;
			CCCV_DISABLE;
		}
	}
	if (cccv_status == CCCV_Finished)
	{
		cccv_battery = CCCV_ADC_SCALE * CCCV_GET_BAT;
		if (cccv_battery <= 0)cccv_battery = 0.1;
	}
}
