/*
 * --------------"��������K60�ײ��"-----------------
 *
 * ����Ӳ��ƽ̨:LPLD_K60 Card
 * ��Ȩ����:�����������µ��Ӽ������޹�˾
 * ��������:http://laplenden.taobao.com
 * ��˾�Ż�:http://www.lpld.cn
 *
 * �ļ���: FML_DiskIO.h
 * ��;: ���̵ײ�ģ����غ����������SDHC�ײ�����
 * ����޸�����: 20130214
 *
 * ������ʹ��Э��:
 *  ��������������ʹ���߿���Դ���룬�����߿��������޸�Դ���롣�����μ�����ע��Ӧ
 *  ���Ա��������ø��Ļ�ɾ��ԭ��Ȩ���������������ο����߿��Լ�ע���ΰ�Ȩ�����ߣ�
 *  ��Ӧ�����ش�Э��Ļ����ϣ�����Դ���롢���ó��۴��뱾��
 */

#ifndef _FML_DISKIO_H_
#define _FML_DISKIO_H_

#include "common.h"

#define _READONLY	0	/* 1: ֻ������ */

/* ���̺������ؽ��ö������ */
typedef enum {
  RES_OK = 0,		/* 0: �ɹ� */
  RES_ERROR,		/* 1: ��/д���� */
  RES_WRPRT,		/* 2: д���� */
  RES_NOTRDY,		/* 3: δ׼���� */
  RES_PARERR,		/* 4: �������� */
  RES_NONRSPNS          /* 5: δ��Ӧ */
} DRESULT;


/* ���̺�������״̬ */
typedef int32	DSTATUS;
/* ����״̬(DSTATUS) */
#define STA_OK		        0x00	/* ״̬���� */
#define STA_NOINIT		0x01	/* Drive not initialized */
#define STA_NODISK		0x02	/* No medium in the drive */
#define STA_PROTECT		0x04	/* Write protected */


/* LPLD_Disk_IOC������������ */
/* ͨ������(Ϊ�����ļ�ϵͳFatFs����) */
#define CTRL_SYNC		0	/* ˢ�´��̻��� (���д����) */
#define GET_SECTOR_COUNT	1	/* ���ý���С (����� f_mkfs()) */
#define GET_SECTOR_SIZE		2	/* ���������С (��Զ�������С (_MAX_SS >= 1024)) */
#define GET_BLOCK_SIZE		3	/* �������������С (����� f_mkfs()) */
#define CTRL_ERASE_SECTOR	4	/* ǿ�Ʋ���һ������ (����� _USE_ERASE) */

/* ͨ������ */
#define CTRL_POWER		5	/* ��ȡ/���õ�Դ״̬ */
#define CTRL_LOCK		6	/* ����/����ý���Ƴ� */
#define CTRL_EJECT		7	/* ����ý�� */

/* MMC/SDC�ض�ioctl���� */
#define MMC_GET_TYPE		10	/* ��ÿ����� */
#define MMC_GET_CSD		11	/* ���CSD */
#define MMC_GET_CID		12	/* ���CID */
#define MMC_GET_OCR		13	/* ���OCR */
#define MMC_GET_SDSTAT		14	/* ���SD��״̬ */

/* ATA/CF�ض�ioctl���� */
#define ATA_GET_REV		20	/* ���F/W���� */
#define ATA_GET_MODEL		21	/* ���ģ������ */
#define ATA_GET_SN		22	/* ������к� */

/* NAND�ض�ioctl���� */
#define NAND_FORMAT		30	/* ���������ʽ */


/* ���̿��ƺ������� */
DSTATUS LPLD_Disk_Initialize(uint8);
DSTATUS LPLD_Disk_Status(uint8);
DRESULT LPLD_Disk_Read(uint8, uint8*, uint32, uint8);
#if _READONLY == 0
DRESULT LPLD_Disk_Write(uint8, const uint8*, uint32, uint8);
#endif
DRESULT LPLD_Disk_IOC(uint8, uint8, void*);


#endif /*_FML_DISKIO_H_*/
