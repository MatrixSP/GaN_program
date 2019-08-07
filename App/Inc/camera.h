#ifndef __CAMERA_H__
#define __CAMERA_H__

#define CAMERA_W            320              //定义摄像头图像宽度
#define CAMERA_H            240              //定义摄像头图像高度
#define CAMERA_R_H          240               //定义摄像头图像高度
#define CAMERA_SIZE         CAMERA_W*CAMERA_H
#define Height              CAMERA_R_H
#define Wide                CAMERA_W
#define BLACK_C 0
#define WHITE_C 254
#define  ThreadHold  150                  //二值化阈值
#define   steer_mid  390     //387
#define  steer_left_max  476
#define  steer_right_max 299
/**********跟踪算法相应参数设置***********/

#define Jump_threhold  50                  //75
#define Interval 7
#define Tracking_displacement  Tracking_NUM/2
#define Tracking_NUM 40
#define Last_line CAMERA_R_H-1

//#define Interval 3
//#define Tracking_displacement 12
//#define Tracking_NUM 24
//#define Last_line CAMERA_R_H-1

#define widths 246                            //底行宽度

void camera(void);
void LP_image(void); 
void yuzhi_cal(void);
void xunxian(void);
void buxian(void);
void MedianFilter();
int qic_Sort(int a, int b, int c);

#endif


/*speed=70

*/
/*speed=75

*/
/*speed=80

*/