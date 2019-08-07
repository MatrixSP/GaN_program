#include "include.h"
#include "IIC.h"
/******************** (C) COPYRIGHT 2011 Ұ��Ƕ��ʽ���������� ********************
 * �ļ���       ��IIC.c
 * ����         �����ٶȼƺ����������ģ��IIC��������
 * ʵ��ƽ̨     ������ӡ�󿪷���
 * ��汾       ������Ұ���
 * Ƕ��ϵͳ     ��
 * ����         ��xuxu
**********************************************************************************/


/************************************************
*  �������ƣ�IIC_start
*  ����˵����IIC start
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
void IIC_start()
{
    SCLout;
    SDAout;
    SCL_L_;
    asm("nop");
    SDA_H_;
    nop5();
    SCL_H_;
    nops();
    SDA_L_;
    nops();
    SCL_L_;
}



/************************************************
*  �������ƣ�IIC_stop
*  ����˵����IIC end//��ֹͣλ SDA=0->1
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
void IIC_stop()
{
    SCLout;
    SDAout;
    SCL_L_;nop5();
    SDA_L_;nop5();
    SCL_H_;nops();
    SDA_H_;nops();
    SCL_L_;
}




/************************************************
*  �������ƣ�IIC_send_byte
*  ����˵����IIC end�ֽڷ��ͳ���
*  ����˵����cΪ�ֽ�
*  �������أ��ޣ������Ǵ�Ӧ��λ
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
void send_byte(unsigned char c)
{
    unsigned char i;
    SCLout;
    SDAout;asm("nop");
    for(i=0;i<8;i++)
    {
        SCL_L_;
        if((c<<i) & 0x80)
            SDA_H_; //�жϷ���λ
        else
            SDA_L_;
        nop5();
        SCL_H_;
        nops();
        SCL_L_;
    }
    nops();
    SDA_H_; //������8bit���ͷ�����׼������Ӧ��λ
    nop5();
    SCL_H_;
    nops(); //sda�����ݼ��Ǵ�Ӧ��λ
    SCL_L_; //�����Ǵ�Ӧ��λ|��Ҫ���ƺ�ʱ��
}



/************************************************
*  �������ƣ�IIC_read_byte
*  ����˵�����ֽڽ��ճ���,������������������
*  ����˵������
*  �������أ�return: uchar��1�ֽ�
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
unsigned char read_byte(void)
{
    unsigned char i;
    unsigned char c;
    SDAin;
    SCLout;
    c=0;
    SCL_L_;
    nop5();
    for(i=0;i<8;i++)
    {
        nop5();
        SCL_L_; //��ʱ����Ϊ�ͣ�׼����������λ
        nops();
        SCL_H_; //��ʱ����Ϊ�ߣ�ʹ��������������Ч
        nop5();
        c<<=1;
        if(SDA_read)
            c+=1; //������λ�������յ����ݴ�c
    }
    SCL_L_;
    return c;
}




/************************************************
*  �������ƣ�IIC_Single_Write
*  ����˵����//д��Ĵ���
*  ����˵����SlaveAddress�豸ID���Ĵ�����ַaddress��thedataΪд������
*  �������أ�return: uchar��1�ֽ�
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
void Single_Write(unsigned char SlaveAddress,unsigned char address, unsigned char thedata)
{
    IIC_start();		//����
    send_byte(SlaveAddress);	//д���豸ID��д�ź�
    send_byte(address);	//X��ַ
    send_byte(thedata);	//д���豸ID������
    IIC_stop();
}



/************************************************
*  �������ƣ�IIC_Single_Read
*  ����˵����//���Ĵ���
*  ����˵����SlaveAddress�豸ID���Ĵ�����ַaddress
*  �������أ�return1���ֽڣ�retΪ��������
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char address)
{
    unsigned char ret = 100;
    IIC_start();		//����
    send_byte(SlaveAddress);	//д���豸ID��д�ź�
    send_byte(address);	//X��ַ
    IIC_start();		//���·��Ϳ�ʼ
    send_byte(SlaveAddress+1);	//д���豸ID������
    ret = read_byte();	//��ȡһ�ֽ�
    IIC_stop();
    return ret;
}



/************************************************
*  �������ƣ�IIC1_start
*  ����˵����IIC1 start
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
void IIC1_start()
{
    SCL1out;
    SDA1out;
    SCL1_L_;
    asm("nop");
    SDA1_H_;
    nop5();
    SCL1_H_;
    nops();
    SDA1_L_;
    nops();
    SCL1_L_;
}



/************************************************
*  �������ƣ�IIC1_stop
*  ����˵����IIC1 end//��ֹͣλ SDA=0->1
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
void IIC1_stop()
{
    SCL1out;
    SDA1out;
    SCL1_L_;nop5();
    SDA1_L_;nop5();
    SCL1_H_;nops();
    SDA1_H_;nops();
    SCL1_L_;
}




/************************************************
*  �������ƣ�IIC1_send_byte
*  ����˵����IIC1 end�ֽڷ��ͳ���
*  ����˵����cΪ�ֽ�
*  �������أ��ޣ������Ǵ�Ӧ��λ
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
void send1_byte(unsigned char c)
{
    unsigned char i;
    SCL1out;
    SDA1out;asm("nop");
    for(i=0;i<8;i++)
    {
        SCL1_L_;
        if((c<<i) & 0x80)
            SDA1_H_; //�жϷ���λ
        else
            SDA1_L_;
        nop5();
        SCL1_H_;
        nops();
        SCL1_L_;
    }
    nops();
    SDA1_H_; //������8bit���ͷ�����׼������Ӧ��λ
    nop5();
    SCL1_H_;
    nops(); //sda�����ݼ��Ǵ�Ӧ��λ
    SCL1_L_; //�����Ǵ�Ӧ��λ|��Ҫ���ƺ�ʱ��
}



/************************************************
*  �������ƣ�IIC1_read_byte
*  ����˵�����ֽڽ��ճ���,������������������
*  ����˵������
*  �������أ�return: uchar��1�ֽ�
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
unsigned char read1_byte(void)
{
    unsigned char i;
    unsigned char c;
    SDA1in;
    SCL1out;
    c=0;
    SCL1_L_;
    nop5();
    for(i=0;i<8;i++)
    {
        nop5();
        SCL1_L_; //��ʱ����Ϊ�ͣ�׼����������λ
        nops();
        SCL1_H_; //��ʱ����Ϊ�ߣ�ʹ��������������Ч
        nop5();
        c<<=1;
        if(SDA1_read)
            c+=1; //������λ�������յ����ݴ�c
    }
    SCL1_L_;
    return c;
}




/************************************************
*  �������ƣ�IIC1_Single_Write
*  ����˵����//д��Ĵ���
*  ����˵����SlaveAddress�豸ID���Ĵ�����ַaddress��thedataΪд������
*  �������أ�return: uchar��1�ֽ�
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
void Single1_Write(unsigned char SlaveAddress,unsigned char address, unsigned char thedata)
{
    IIC1_start();		//����
    send1_byte(SlaveAddress);	//д���豸ID��д�ź�
    send1_byte(address);	//X��ַ
    send1_byte(thedata);	//д���豸ID������
    IIC1_stop();
}

/************************************************
*  �������ƣ�IIC1_Single_Read
*  ����˵����//���Ĵ���
*  ����˵����SlaveAddress�豸ID���Ĵ�����ַaddress
*  �������أ�return1���ֽڣ�retΪ��������
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
unsigned char Single1_Read(unsigned char SlaveAddress,unsigned char address)
{
    unsigned char ret = 100;
    IIC1_start();		//����
    send1_byte(SlaveAddress);	//д���豸ID��д�ź�
    send1_byte(address);	//X��ַ
    IIC1_start();		//���·��Ϳ�ʼ
    send1_byte(SlaveAddress+1);	//д���豸ID������
    ret = read1_byte();	//��ȡһ�ֽ�
    IIC1_stop();
    return ret;
}


/************************************************
*  �������ƣ�
*  ����˵����MMA845x��ʼ������ʼ��Ϊָ��ģʽ
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
void MMA845x_init()
{
    //Single_Write(MMA845x_IIC_ADDRESS , MMACTRL_REG1,(Single_Read(MMA845x_IIC_ADDRESS,MMACTRL_REG1)& ~FREAD_MASK));
    nops(); nops(); nops(); nops(); nops(); nops();
    Single_Write(MMA845x_IIC_ADDRESS,MMACTRL_REG1,ASLP_RATE_20MS+DATA_RATE_5MS);      //0x38  0x2A  0x00+0x10
    nops(); nops(); nops(); nops(); nops(); nops();
    Single_Write(MMA845x_IIC_ADDRESS,XYZ_DATA_CFG_REG, FULL_SCALE_2G); //2G                   0x0E  0x00
    nops(); nops(); nops(); nops(); nops(); nops();
    Single_Write(MMA845x_IIC_ADDRESS,HP_FILTER_CUTOFF_REG, PULSE_LPF_EN_MASK ); //��ͨ�˲�    0x0F  0x10
    nops(); nops(); nops(); nops(); nops(); nops();
    Single_Write(MMA845x_IIC_ADDRESS,MMACTRL_REG1, ACTIVE_MASK);          //����״̬          0x2A  0x01
    nops(); nops(); nops(); nops(); nops(); nops();
}


/************************************************
*  �������ƣ�Get_mma_average
*  ����˵�����õ����ٶ�ֵ
*  ����˵����nΪ�����Σ�AxisΪ�ĸ���'X''Y''Z'
*  �������أ�Ϊ-1~1��ֵ
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
double mma_switch(unsigned char h,unsigned char l)
{
    double ret;
    unsigned short int V1,h_l;
    int sign;
    h_l=(h<<8u)+l;
    //14λ
    if(h>0x7F)
    {
        sign=-1;         //****************�Ҹ��ˣ�Ӧ����-1
        V1=(~(h_l>>2)+1)&0X3FFF;
    }
    else
    {
        sign=1;
        V1=(h_l>>2)&0X3FFF;
    }
    ret=sign*(((double)V1)/0xfff);

    //12λ
//    if(h>0x7F)
//    {
//        sign=-1;         //****************�Ҹ��ˣ�Ӧ����-1
//        V1=(~(h_l>>4)+1)&0X0FFF;
//    }
//    else
//    {
//        sign=1;
//        V1=(h_l>>4)&0X0FFF;
//    }
//    ret=sign*(((double)V1)/0x3ff);
    return ret;
}




/************************************************
*  �������ƣ�mma_switch
*  ����˵�������ٶȼ�ֱ�ӵõ���������-1~1��ת��
*  ����˵����h��lΪ��Ҫת����ֵ
*  �������أ����ٶȼƵ�ֵ-1~1��x=���ٶ�/g��
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
double Get_mma8451_once(unsigned char Axis)
{
    unsigned char h,l;
    double temp = 0;
    unsigned char MSB,LSB;
    if(Axis=='X')  //����
    {
  	MSB=OUT_X_MSB_REG;
  	LSB=OUT_X_LSB_REG;
    }
    else if(Axis=='Y')   //cos
    {
  	MSB=OUT_Y_MSB_REG;
  	LSB=OUT_Y_LSB_REG;
    }
    else if(Axis=='Z')   //sin
    {
  	MSB=OUT_Z_MSB_REG;
  	LSB=OUT_Z_LSB_REG;
    }
    


    h=Single_Read(MMA845x_IIC_ADDRESS,MSB);
    l=Single_Read(MMA845x_IIC_ADDRESS,LSB);
    temp=mma_switch(h,l);
    if(temp>=0.9999)
    {
        temp=0.9999;//���ص��Ǵ�������־
        //temp=temp/1.00;
    }
    else if(temp<=-0.9999)
    {
        temp=-0.9999;//���ص��Ǵ�������־
        //temp=temp/1.00;			//���ٶȼƵ����ֵΪ1.02
    }
    return(temp);
}



/************************************************
*  �������ƣ�Init_L3G4200D
*  ����˵���������ǵĳ�ʼ������
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
void Init_L3G4200D(void)
{
  /*
  i2c_init(I2C1,400*1000);
  
  i2c_write_reg(I2C1, L3G_Address, CTRL_REG1, 0x0f);
  i2c_write_reg(I2C1, L3G_Address, CTRL_REG2, 0x00);
  i2c_write_reg(I2C1, L3G_Address, CTRL_REG3, 0x08);
  i2c_write_reg(I2C1, L3G_Address, CTRL_REG4, 0x30);
  i2c_write_reg(I2C1, L3G_Address, CTRL_REG5, 0x00);
  */
   
   Single1_Write(0xD2,0x20, 0x0f);   //
   
   Single1_Write(0xD2,0x21, 0x00);   //
  
   Single1_Write(0xD2,0x22, 0x08);   //
   
   Single1_Write(0xD2,0x23, 0x30);  //+-2000dps
  
   Single1_Write(0xD2,0x24, 0x00);
    

}



/************************************************
*  �������ƣ�Get_Gyro
*  ����˵������ȡL3G4200D����
*  ����˵����nΪ�����Σ�AxisΪ�ĸ���,'X''Y''Z'
*  �������أ����ٶȵ�ֵ
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
long int Get_Gyro(unsigned char n,unsigned char Axis) //nΪ������
{
    int16 h = 0,l = 0;
    //long int sum=0;
    int16 single=0;
    //unsigned char i;
    int8 MSB,LSB;
    if(Axis=='X')  //����
    {
  	MSB=OUT_X_H;
  	LSB=OUT_X_L;
    }
    else if(Axis=='Y')   //cos
    {
  	MSB=OUT_Y_H;
  	LSB=OUT_Y_L;
    }
    else if(Axis=='Z')   //sin
    {
  	MSB=OUT_Z_H;
  	LSB=OUT_Z_L;
    }

        h = Single1_Read(0xD2, MSB);
        l = Single1_Read(0xD2, LSB);
        single = (h<<8) + l;
        
    
    return(single);
}
