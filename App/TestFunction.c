#include "include.h"
/*===================================================================
功能：彩灯状态
输入：二值逻辑，0表示亮，1表示不亮，red，blue，green分别对应三原色
===================================================================*/
# define Red_On gpio_set(PTA10, 0);
# define Red_Off gpio_set(PTA10, 1);
# define Green_On gpio_set(PTE8, 0);
# define Green_Off gpio_set(PTE8, 1);
# define Blue_On gpio_set(PTE9, 0);
# define Blue_Off gpio_set(PTE9, 1);

void Light_State(int red, int blue, int green)
{
	int command = red * 4 + blue * 2 + green;

	switch (command)
	{
	case 0:
		Red_Off
		Blue_Off
		Green_Off
		break;
	case 1:
		Red_Off
		Blue_Off
		Green_On
		break;
	case 2:
		Red_Off
		Blue_On
		Green_Off
		break;
	case 3:
		Red_Off
		Green_On
		Blue_On
		break;
	case 4:
		Red_On
		Green_Off
		Blue_Off
		break;
	case 5:
		Red_On
		Blue_Off
		Green_On
		break;
	case 6:
		Red_Off
		Blue_On
		Green_On
		break;
	case 7:
		Red_On
		Blue_On
		Green_On
		break;
	default:
		Red_Off
		Blue_Off
		Green_Off
		break;
	}
}

/*===================================================================
功能：串口发送数据用于Matlab数据处理
===================================================================*/
void DATA_SEND(long num)
{
	int weishu = 0;
	long num_buff = num;
	long buff = 1;
	int index = 0;

	if (num < 0)
	{
		uart_putchar(UART5, '-');
		num = -num;
	}

	if (num_buff == 0)
	{
		weishu = 1;
		buff = 10;
	}

	while (num_buff != 0)
	{
		num_buff /= 10;
		weishu++;
		buff *= 10;
	}
	buff /= 10;

	for (index = 0; index < weishu; index++)
	{
		uart_putchar(UART5, ((num / buff) % 10) + '0');
		buff /= 10;
	}
	uart_putchar(UART5, ' ');
}

