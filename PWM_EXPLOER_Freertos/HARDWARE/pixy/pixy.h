#ifndef _PIXY_H_
#define _PIXY_H_
#include "sys.h"
#define SendBuff_Size   18
typedef struct Pixy_Color//单色块位置大小信息
{
uint16_t Pixy_Color_Sig;//1-7标记号码
uint16_t Pixy_Color_PosX;  //0 to 319
uint16_t Pixy_Color_PosY;  //0 to 319
uint16_t Pixy_Color_Width; //1 to 320
uint16_t Pixy_Color_Height;//1 to 320
}Pixy_Color;

typedef struct Pixy_ColorCode//色调位置大小信息
{
uint16_t Pixy_ColorCode_Sig;//同上
uint16_t Pixy_ColorCode_PosX;
uint16_t Pixy_ColorCode_PosY;
uint16_t Pixy_ColorCode_Width;
uint16_t Pixy_ColorCode_Height;
uint16_t Pixy_ColorCode_Angle;//
}Pixy_ColorCode;

extern float x,y;
extern u8 j ;
extern unsigned char Pixy_Data[SendBuff_Size];
extern int USART2_FAIL ;	//通信失败标志、帧头错误置高
extern int USART2_Half_OK;
extern unsigned char Raw_Data[19];
extern char USART2_Flag ;
extern u16 counter;

void base_func1(float x,float y);
void base_func2(float x,float y);

void base_func3(float x,float y);
void base_func3_step1(float x,float y);//从1-4 
void base_func3_step2(float x,float y);// 从4—5

void base_func4(float x,float y);

void advance_func1(float x,float y);
void advance_func2(float x,float y);
void advance_func3(float x,float y);

static float x_group[] = {226,160,93,
													227,161,91,
													225,159,92};
static float y_group[] = {161,164,162,
													96,96,95,
													28,28,31};

static int fb_time[] = {
											10,10,10
									};
static float fb_x[] = {
					135,120,120
				};
static float fb_y[] = {
					170,150,140
				};

#endif
