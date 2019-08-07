/*
 * --------------"拉普兰德K60底层库"示例工程-----------------
 *
 * 测试硬件平台:  LPLD_K60 Card
 * 版权所有:      北京拉普兰德电子技术有限公司
 * 网络销售:      http://laplenden.taobao.com
 * 公司门户:      http://www.lpld.cn
 *
 * 说明:    本工程基于"拉普兰德K60底层库"开发，
 *          所有开源驱动代码均在"LPLD"文件夹下，调用说明见文档[#LPLD-003-N]
 *          本工程所用文件系统基于FatFs开源文件系统，仅修改了调用Disk底层函数名
 *
 * 文件名:  LPLD_SDHC_FatFs.c
 * 用途:    SD卡文件系统测试程序，请格式化SD卡后再使用。
 *          本程序在SD卡目录下建立txt文档，并写入、读出数据。
 *          数据在K60默认串口5输出，波特率115200.
 *          如果运行失败，请重新插拔SD卡再运行.
 *
 */

#include "common.h"
#include "uart.h"
// 需要包含stdio头文件
#include <stdio.h>
// 只需包含文件系统头文件即可，其内部自动调用SDHC底层驱动
#include "ff.h"   

// 打印文件返回代码
void die(FRESULT rc)
{
  printf("错误代码 rc=%u.\n", rc);
  for (;;) ;
}


/********************************************************************/
void main (void)
{

  int i;
  // 以下变量定义均采用FatFs文件系统变量类型
  FRESULT rc;			/* 结果代码 */
  FATFS fatfs;			/* 文件系统对象 */
  FIL fil;			/* 文件对象 */
  UINT bw, br;
  BYTE buff[128];
  
  f_mount(0, &fatfs);		/* 注册一个磁盘工作区 */
  
  printf("当前磁盘大小: %d Bytes\n", fatfs.fsize*_MAX_SS);
  //创建一个新的txt文档
  printf("新建一个文件 (LPLD_FatFs2.TXT).\n");
  rc = f_open(&fil, "LPLD_FatFs2.TXT", FA_WRITE | FA_CREATE_ALWAYS);
  if (rc) die(rc);
  
  //向新创建的文档中写入"Hello LPLD!"并换行
  printf("写入文本数据. (Hello LPLD!)\n");
  rc = f_write(&fil, "Hello LPLD!\r\n", 13, &bw);
  if (rc) die(rc);
  printf("共写入 %u Bytes.\n", bw);
  
  //关闭新建的文件
  printf("关闭文件.\n\n");
  rc = f_close(&fil);
  if (rc) die(rc);
  
  //打开刚才新建的文件
  printf("打开一个文件 (LPLD_FatFs2.TXT).\n");
  rc = f_open(&fil, "LPLD_FatFs2.TXT", FA_READ);
  if (rc) die(rc);
  
  //打印出文件内的内容
  printf("打印此文件内容.\n");
  for (;;) 
  {
    rc = f_read(&fil, buff, sizeof(buff), &br);	/* 读取文件的一块 */
    if (rc || !br) break;			/* 错误或读取完毕 */
    for (i = 0; i < br; i++)		        /* 输出读取的字节数据 */
      uart_putchar(TERM_PORT, buff[i]);
  }
  if (rc) die(rc);
  
  //关闭文件
  printf("\n关闭文件.\n");
  rc = f_close(&fil);
  if (rc) die(rc);
  
  printf("文件系统测试完毕.\n");
  
  while(1)
  {
  } 
}
/********************************************************************/

// 用户自定义的为FatFs系统提供实时时间的函数
DWORD get_fattime (void)
{
  return ((DWORD)(2013 - 1980) << 25)	//2013年
       | ((DWORD)3 << 21)              //3月
       | ((DWORD)15 << 16)              //15日
       | ((DWORD)0 << 11)
       | ((DWORD)0 << 5)
       | ((DWORD)0 >> 1);
}