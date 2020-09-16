#ifndef __KALMANFILTER_H
#define __KALMANFILTER_H

#include "GLOBAL.h"

typedef struct
{
	float x;
	float y;
	float z;
}SI_F_XYZ;

typedef struct
{
	signed short x;
	signed short y;
	signed short z;
}S16_XYZ;

typedef struct 
{
    SI_F_XYZ deg_s;
    SI_F_XYZ rad_s;
    SI_F_XYZ acc_g;

	float att_acc_factor;
    float fix_acc_factor;
    
}_Mpu6050_data;

extern _Mpu6050_data Mpu;

extern SI_F_XYZ acc_raw_f;
extern SI_F_XYZ gyro_raw_f;

extern SI_F_XYZ gyro_offset;          //陀螺仪零偏数据存储
extern S16_XYZ gyro_raw_p; 
extern S16_XYZ acc_raw_p;

extern SI_F_XYZ acc_att_lpf;
extern S16_XYZ gyro_raw;

typedef struct
{
    float input_data[3];
    float output_data[3];
}_Butterworth_data;

typedef struct
{
    const float a[3];
    const float b[3];
}_Butterworth_parameter;

float butterworth_lpf(float now_input,_Butterworth_data *buffer, _Butterworth_parameter *parameter);

float Kalman_Filter(double angle_m, double gyro_m);
float QuaternionMethod(float gx, float gy, float gz, float ax, float ay, float az);

#define PI                      3.1415926535898f
#define gyro_raw_to_deg_s       0.007633587786f   //+-250°/s:131LSB/°/s()   +-500°/s:65.5LSB/°/s   +-1000°/s:32.8LSB/°/s    +-2000°/s:16.4LSB/°/s(本次所以)
#define acc_raw_to_g            0.000061035156f    //+-2g : 16384LSB/g     +-4g : 8192LSB/g   +-8g : 4096LSB/g(本次所用)   +-16g : 2048LSB/g  

#define deg_to_rad              (PI / 180.0f)
#define rad_to_angle            (180.0f / PI)                    
#define gyro_raw_to_radian_s	(gyro_raw_to_deg_s * deg_to_rad)

void get_gyro_raw(void);
void get_rad_s(SI_F_XYZ *gyro_in,SI_F_XYZ *gyro_out);
void get_deg_s(SI_F_XYZ *gyro_in,SI_F_XYZ *gyro_deg_out);

void get_acc_raw(void);
void acc_iir_lpf(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out,float lpf_factor);
void get_iir_factor(float *out_factor,float Time, float Cut_Off);
void get_acc_g(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out);

#endif 