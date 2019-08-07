/*
 * --------------"��������K60�ײ��"ʾ������-----------------
 *
 * ����Ӳ��ƽ̨:  LPLD_K60 Card
 * ��Ȩ����:      �����������µ��Ӽ������޹�˾
 * ��������:      http://laplenden.taobao.com
 * ��˾�Ż�:      http://www.lpld.cn
 *
 * ˵��:    �����̻���"��������K60�ײ��"������
 *          ���п�Դ�����������"LPLD"�ļ����£�����˵�����ĵ�[#LPLD-003-N]
 *          �����������ļ�ϵͳ����FatFs��Դ�ļ�ϵͳ�����޸��˵���Disk�ײ㺯����
 *
 * �ļ���:  LPLD_SDHC_FatFs.c
 * ��;:    SD���ļ�ϵͳ���Գ������ʽ��SD������ʹ�á�
 *          ��������SD��Ŀ¼�½���txt�ĵ�����д�롢�������ݡ�
 *          ������K60Ĭ�ϴ���5�����������115200.
 *          �������ʧ�ܣ������²��SD��������.
 *
 */

#include "common.h"
#include "uart.h"
// ��Ҫ����stdioͷ�ļ�
#include <stdio.h>
// ֻ������ļ�ϵͳͷ�ļ����ɣ����ڲ��Զ�����SDHC�ײ�����
#include "ff.h"   

// ��ӡ�ļ����ش���
void die(FRESULT rc)
{
  printf("������� rc=%u.\n", rc);
  for (;;) ;
}


/********************************************************************/
void main (void)
{

  int i;
  // ���±������������FatFs�ļ�ϵͳ��������
  FRESULT rc;			/* ������� */
  FATFS fatfs;			/* �ļ�ϵͳ���� */
  FIL fil;			/* �ļ����� */
  UINT bw, br;
  BYTE buff[128];
  
  f_mount(0, &fatfs);		/* ע��һ�����̹����� */
  
  printf("��ǰ���̴�С: %d Bytes\n", fatfs.fsize*_MAX_SS);
  //����һ���µ�txt�ĵ�
  printf("�½�һ���ļ� (LPLD_FatFs2.TXT).\n");
  rc = f_open(&fil, "LPLD_FatFs2.TXT", FA_WRITE | FA_CREATE_ALWAYS);
  if (rc) die(rc);
  
  //���´������ĵ���д��"Hello LPLD!"������
  printf("д���ı�����. (Hello LPLD!)\n");
  rc = f_write(&fil, "Hello LPLD!\r\n", 13, &bw);
  if (rc) die(rc);
  printf("��д�� %u Bytes.\n", bw);
  
  //�ر��½����ļ�
  printf("�ر��ļ�.\n\n");
  rc = f_close(&fil);
  if (rc) die(rc);
  
  //�򿪸ղ��½����ļ�
  printf("��һ���ļ� (LPLD_FatFs2.TXT).\n");
  rc = f_open(&fil, "LPLD_FatFs2.TXT", FA_READ);
  if (rc) die(rc);
  
  //��ӡ���ļ��ڵ�����
  printf("��ӡ���ļ�����.\n");
  for (;;) 
  {
    rc = f_read(&fil, buff, sizeof(buff), &br);	/* ��ȡ�ļ���һ�� */
    if (rc || !br) break;			/* ������ȡ��� */
    for (i = 0; i < br; i++)		        /* �����ȡ���ֽ����� */
      uart_putchar(TERM_PORT, buff[i]);
  }
  if (rc) die(rc);
  
  //�ر��ļ�
  printf("\n�ر��ļ�.\n");
  rc = f_close(&fil);
  if (rc) die(rc);
  
  printf("�ļ�ϵͳ�������.\n");
  
  while(1)
  {
  } 
}
/********************************************************************/

// �û��Զ����ΪFatFsϵͳ�ṩʵʱʱ��ĺ���
DWORD get_fattime (void)
{
  return ((DWORD)(2013 - 1980) << 25)	//2013��
       | ((DWORD)3 << 21)              //3��
       | ((DWORD)15 << 16)              //15��
       | ((DWORD)0 << 11)
       | ((DWORD)0 << 5)
       | ((DWORD)0 >> 1);
}