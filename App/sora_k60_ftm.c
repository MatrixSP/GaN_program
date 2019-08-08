#include "include.h"

extern FTM_MemMapPtr FTMN[];

//deadtime: ns
void FTM_COMP_init(FTMn_e ftmn, FTM_Gn_e gn, uint32 freq, uint32 duty, float deadtime)
{
	uint32 clk_hz;
	uint16 mod;
	uint8  ps;
	uint16 cv;
	FTM_CHn_e ch1, ch2;
	float deadct;
	uint32 deadcn;
	uint8 deaddiv;

	ASSERT((ftmn == FTM0) || ((ftmn == FTM1 || ftmn == FTM2) && (gn <= FTM_G3)));  //检查传递进来的通道是否正确
	ASSERT(freq <= (bus_clk_khz * 1000 >> 1));                                           //用断言检测 频率 是否正常 ,频率必须小于时钟二分之一

	if (ftmn == FTM0)
	{
		switch (gn)
		{
		case FTM_G0:
			ch1 = FTM_CH0;
			ch2 = FTM_CH1;

			break;
		case FTM_G1:
			ch1 = FTM_CH2;
			ch2 = FTM_CH3;
			break;
		case FTM_G2:
			ch1 = FTM_CH4;
			ch2 = FTM_CH5;
			break;
		case FTM_G3:
			ch1 = FTM_CH6;
			ch2 = FTM_CH7;
			break;
		default:
			break;
		}
	}
	else
	{
		ch1 = FTM_CH0;
		ch2 = FTM_CH1;
	}


	/******************* 开启时钟 和 复用IO口*******************/
	switch (ftmn)
	{
	case FTM0:
		SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;       //使能FTM0时钟
		switch (gn)
		{
		case FTM_G0:
			if (FTM0_CH0 == PTC1)
			{
				port_init(FTM0_CH0, ALT4);
			}
			else if (FTM0_CH0 == PTA3)
			{
				port_init(FTM0_CH0, ALT3);
			}
			else
			{
				ASSERT(0);                      //设置管脚有误？
			}
			if (FTM0_CH1 == PTC2)
			{
				port_init(FTM0_CH1, ALT4);
			}
			else if (FTM0_CH1 == PTA4)
			{
				port_init(FTM0_CH1, ALT3);
			}
			else
			{
				ASSERT(0);                      //设置管脚有误？
			}
			break;

		case FTM_G1:
			if (FTM0_CH2 == PTC3)
			{
				port_init(FTM0_CH2, ALT4);
			}
			else if (FTM0_CH2 == PTA5)
			{
				port_init(FTM0_CH2, ALT3);
			}
			else
			{
				ASSERT(0);                      //设置管脚有误？
			}
			if (FTM0_CH3 == PTC4)
			{
				port_init(FTM0_CH3, ALT4);
			}
			else if (FTM0_CH3 == PTA6)
			{
				port_init(FTM0_CH3, ALT3);
			}
			else
			{
				ASSERT(0);                      //设置管脚有误？
			}
			break;

		case FTM_G2:
			if (FTM0_CH4 == PTD4)
			{
				port_init(FTM0_CH4, ALT4);
			}
			else if (FTM0_CH4 == PTA7)
			{
				port_init(FTM0_CH4, ALT3);
			}
			else
			{
				ASSERT(0);                      //设置管脚有误？
			}
			if (FTM0_CH5 == PTD5)
			{
				port_init(FTM0_CH5, ALT4);
			}
			else if (FTM0_CH5 == PTA0)
			{
				port_init(FTM0_CH5, ALT3);
			}
			else
			{
				ASSERT(0);                      //设置管脚有误？
			}
			break;

		case FTM_G3:
			if (FTM0_CH6 == PTD6)
			{
				port_init(FTM0_CH6, ALT4);
			}
			else if (FTM0_CH6 == PTA1)
			{
				port_init(FTM0_CH6, ALT3);
			}
			else
			{
				ASSERT(0);                      //设置管脚有误？
			}
			if (FTM0_CH7 == PTD7)
			{
				port_init(FTM0_CH7, ALT4);
			}
			else if (FTM0_CH7 == PTA2)
			{
				port_init(FTM0_CH7, ALT3);
			}
			else
			{
				ASSERT(0);                      //设置管脚有误？
			}
			break;
		default:
			return;
		}
		break;

	case FTM1:
		SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;       //使能FTM1时钟
		if ((FTM1_CH0 == PTA8) || (FTM1_CH0 == PTA12) || (FTM1_CH0 == PTB0))
		{
			port_init(FTM1_CH0, ALT3);
		}
		else
		{
			ASSERT(0);                      //设置管脚有误？
		}
		if ((FTM1_CH1 == PTA9) || (FTM1_CH1 == PTA13) || (FTM1_CH1 == PTB1))
		{
			port_init(FTM1_CH1, ALT3);
		}
		else
		{
			ASSERT(0);                      //设置管脚有误？
		}
		break;

	case FTM2:
		SIM_SCGC3 |= SIM_SCGC3_FTM2_MASK;                           //使能FTM2时钟
		if ((FTM2_CH0 == PTA10) || (FTM2_CH0 == PTB18))
		{
			port_init(FTM2_CH0, ALT3);
		}
		else
		{
			ASSERT(0);                      //设置管脚有误？
		}
		if ((FTM2_CH1 == PTA11) || (FTM2_CH1 == PTB19))
		{
			port_init(FTM2_CH1, ALT3);
		}
		else
		{
			ASSERT(0);                      //设置管脚有误？
		}
		break;
	default:
		break;
	}

	// 以 CPWMS = 1 ，即双边捕捉脉冲为例
	clk_hz = (bus_clk_khz * 1000) >> 1; // bus频率 / 2

	mod = (clk_hz >> 16) / freq;      // 临时用 mod 缓存一下
	ps = 0;
	while ((mod >> ps) >= 1)             // 等 (mod >> ps) < 1 才退出 while 循环 ，即求 PS 的最小值
	{
		ps++;
	}

	ASSERT(ps <= 0x07);         // 断言， PS 最大为 0x07 ，超过此值，则 PWM频率设置过低，或 Bus 频率过高

	mod = (clk_hz >> ps) / freq;// 求 MOD 的值

	switch (ftmn)                // 初值 CNTIN 设为0 ，脉冲宽度：CnV - CNTIN ，即 CnV 就是 脉冲宽度了。
	{
		// EPWM的周期 ： MOD - CNTIN + 0x0001 == MOD - 0 + 1
		// 则 CnV = (MOD - 0 + 1) * 占空比 = (MOD - 0 + 1) * duty/ FTM_PRECISON
	case FTM0:
		cv = (duty * (mod - 0 + 1)) / FTM0_PRECISON;
		break;

	case FTM1:
		cv = (duty * (mod - 0 + 1)) / FTM1_PRECISON;
		break;

	case FTM2:
		cv = (duty * (mod - 0 + 1)) / FTM2_PRECISON;
		break;

	default:
		break;
	}

	deadct = 1 / (bus_clk_khz * 0.000001);
	deadcn = deadtime / deadct;
	if (deadcn <= 0x3f)
	{
		deaddiv = 0;
	}
	else if (deadcn <= (0x3f * 4))
	{
		deaddiv = 2;
		deadcn = deadcn / 4;
	}
	else if (deadcn <= (0x3f * 16))
	{
		deaddiv = 3;
		deadcn = deadcn / 16;
	}
	else
	{
		deaddiv = 3;
		deadcn = 0x3f;
	}

	FTM_CONF_REG(FTMN[ftmn]) = FTM_CONF_BDMMODE(3);
	FTM_FMS_REG(FTMN[ftmn]) = 0x00;
	FTM_MODE_REG(FTMN[ftmn]) = FTM_MODE_WPDIS_MASK | FTM_MODE_FTMEN_MASK;

	FTM_MOD_REG(FTMN[ftmn]) = mod;                        //模数, EPWM的周期为 ：MOD - CNTIN + 0x0001
	FTM_CnSC_REG(FTMN[ftmn], ch1) = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM_CnSC_REG(FTMN[ftmn], ch2) = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;

	FTM_COMBINE_REG(FTMN[ftmn]) |= (FTM_COMBINE_COMP0_MASK | FTM_COMBINE_DTEN0_MASK) << (gn * 8);
	
	FTM_DEADTIME_REG(FTMN[ftmn]) = FTM_DEADTIME_DTPS(deaddiv) | FTM_DEADTIME_DTVAL(deadcn);
	FTM_CnV_REG(FTMN[ftmn], ch1) = cv;
	FTM_CNTIN_REG(FTMN[ftmn]) = 0;

	FTM_SC_REG(FTMN[ftmn]) = (0
		| FTM_SC_CPWMS_MASK
		| FTM_SC_PS(ps)             //分频因子，分频系数 = 2^PS
		| FTM_SC_CLKS(1)            //时钟选择， 0：没选择时钟，禁用； 1：bus 时钟； 2：MCGFFCLK； 3：EXTCLK（ 由SIM_SOPT4 选择输入管脚 FTM_CLKINx）
		);

}