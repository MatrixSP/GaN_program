/*
 * --------------"��������K60�ײ��"-----------------
 *
 * ����Ӳ��ƽ̨:LPLD_K60 Card
 * ��Ȩ����:�����������µ��Ӽ������޹�˾
 * ��������:http://laplenden.taobao.com
 * ��˾�Ż�:http://www.lpld.cn
 *
 * �ļ���: FML_DiskIO.c
 * ��;: ���̵ײ�ģ����غ����������SDHC�ײ�����
 * ����޸�����: 20130214
 *
 * ������ʹ��Э��:
 *  ��������������ʹ���߿���Դ���룬�����߿��������޸�Դ���롣�����μ�����ע��Ӧ
 *  ���Ա��������ø��Ļ�ɾ��ԭ��Ȩ���������������ο����߿��Լ�ע���ΰ�Ȩ�����ߣ�
 *  ��Ӧ�����ش�Э��Ļ����ϣ�����Դ���롢���ó��۴��뱾��
 */

#include "common.h"
#include "HAL_SDHC.h"
#include "FML_DiskIO.h"

extern SDCARD_STRUCT_PTR        sdcard_ptr;

/******************************************************************************
*
*   ��������
*
******************************************************************************/


/*
 * LPLD_Disk_Initialize
 * ���̳�ʼ��
 * 
 * ����:
 *    drv--������̺ţ�ֻ��Ϊ0
 *
 * ���:
 *    RES_OK--�ɹ�
 *    RES_ERROR--��/д����
 *    RES_WRPRT--д����
 *    RES_NOTRDY--δ׼����
 *    RES_PARERR--��������
 *    RES_NONRSPNS--δ��Ӧ 
 */

DSTATUS LPLD_Disk_Initialize(uint8 drv)
{
  if(drv)
    return RES_PARERR;
 
  return LPLD_SDHC_InitCard();
}

/*
 * LPLD_Disk_Status
 * ���ش���״̬
 * 
 * ����:
 *    drv--������̺ţ�ֻ��Ϊ0
 *
 * ���:
 *    RES_OK--�ɹ�
 *    RES_ERROR--��/д����
 *    RES_WRPRT--д����
 *    RES_NOTRDY--δ׼����
 *    RES_PARERR--��������
 *    RES_NONRSPNS--δ��Ӧ 
 */
DSTATUS LPLD_Disk_Status(uint8 drv)
{
  return sdcard_ptr->STATUS;
}


/*
 * LPLD_Disk_Read
 * �����̵�һ����������
 * 
 * ����:
 *    drv--������̺ţ�ֻ��Ϊ0
 *    buff--���ݶ�������ĵ�ַָ��
 *    sector--������ʼ��
 *    count--������������
 *
 * ���:
 *    RES_OK--�ɹ�
 *    RES_ERROR--��/д����
 *    RES_WRPRT--д����
 *    RES_NOTRDY--δ׼����
 *    RES_PARERR--��������
 *    RES_NONRSPNS--δ��Ӧ 
 */
DRESULT LPLD_Disk_Read(uint8 drv, uint8* buff, uint32 sector, uint8 count)
{
  if(drv || (count == 0))
    return RES_PARERR;

  return LPLD_SDHC_ReadBlocks(buff, sector, count);
}

//�������ϵͳΪֻ�����򲻱������º���
#if	_READONLY == 0
/*
 * LPLD_Disk_Write
 * д���̵�һ����������
 * 
 * ����:
 *    drv--������̺ţ�ֻ��Ϊ0
 *    buff--���ݶ�������ĵ�ַָ��
 *    sector--������ʼ��
 *    count--������������
 *
 * ���:
 *    RES_OK--�ɹ�
 *    RES_ERROR--��/д����
 *    RES_WRPRT--д����
 *    RES_NOTRDY--δ׼����
 *    RES_PARERR--��������
 *    RES_NONRSPNS--δ��Ӧ 
 */

DRESULT LPLD_Disk_Write(uint8 drv, const uint8* buff, uint32 sector, uint8 count)
{
  if(drv || (count == 0))
    return RES_PARERR;
  
  return LPLD_SDHC_WriteBlocks((uint8*)buff, sector, count);
}
#endif


/*
 * LPLD_Disk_IOC
 * ���̹��ܿ��ƺ���
 * 
 * ����:
 *    drv--������̺ţ�ֻ��Ϊ0
 *    ctrl--��������
 *    buff--IO�����������ݵ�ָ��
 *
 * ���:
 *    RES_OK--�ɹ�
 *    RES_ERROR--��/д����
 *    RES_WRPRT--д����
 *    RES_NOTRDY--δ׼����
 *    RES_PARERR--��������
 *    RES_NONRSPNS--δ��Ӧ 
 */
DRESULT LPLD_Disk_IOC(uint8 drv, uint8 ctrl, void* buff)
{
  DRESULT result = RES_OK;
  
  if(drv)
    return RES_PARERR;
  
  
  
  switch(ctrl)
  {
    case CTRL_SYNC:
      /*
      ȷ�����������Ѿ����д��������Ĵ���. 
      ������IOģ����һ����д���棬��������������. ���������ֻ��ģʽʹ��.
      */
      // ��POLLINGģʽ�У�����д��������ɡ�
      break;
    case GET_SECTOR_SIZE:
      /*
      ��WORD��ָ���������ʽ����������С.
      ����������ڿɱ�������С������, 
      _MAX_SS Ϊ 512.
      */
      if(buff == NULL)
        result = RES_PARERR;
      else
        *(uint32*)buff = IO_SDCARD_BLOCK_SIZE;
      
      break;
    case GET_SECTOR_COUNT:
      /*
      ��UINT32��ָ���������ʽ���ش��̵Ŀ���������. 
      ���������f_mkfs���������Ծ����ɴ������ľ�. 
      */
      if(buff == NULL)
        result = RES_PARERR;
      else
        *(uint32*)buff = sdcard_ptr->NUM_BLOCKS;
      break;
    case GET_BLOCK_SIZE:
      /*
      ��UINT32���͵�ָ��������ط���flash�ڴ��в�����������.
      �Ϸ�����ֵΪ2��1��32768�η�.
      ����1���������С������豸δ֪.
      ���������f_mkfs�������ò���ͼ�������������߽�������ݶ���.
      */
      result = RES_PARERR;
      break;
    case CTRL_ERASE_SECTOR:
      /*
      ������UINT32����ָ������ָ����flash�ڴ�,{<start sector>, <end sector>}.
      �������Ϊ��flash�ڴ�,���������Ч.
      FatFsϵͳ��������,�������ʧ��Ҳ����Ӱ���ļ�����.
      ��_USE_ERASEΪ1ʱ�ƶ�һ����������ô�����.
      */
      result = RES_PARERR;
      break;
    default:
      return RES_PARERR;
    
  }
    



  return result;
}

