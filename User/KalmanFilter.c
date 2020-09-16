#include "KalmanFilter.h"
#include "math.h"

S16_XYZ acc_raw = {0};                  //���ٶȼ�ԭʼ���ݴ洢
S16_XYZ gyro_raw = {0};                 //������ԭʼ���ݴ洢
SI_F_XYZ acc_raw_f = {0};
SI_F_XYZ gyro_raw_f = {0};

SI_F_XYZ gyro_offset = {0,0,0} ;         //��������ƫ���ݴ洢
S16_XYZ gyro_raw_p = {0,0,0};
S16_XYZ acc_raw_p  = {0,0,0};

SI_F_XYZ acc_att_lpf = {0};

float angle;
float q_bias;
float angle_err;
float Pdot[4] = { 0, 0, 0, 0};
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float  PCt_0, PCt_1,E, K_0, K_1, t_0, t_1;
float angle_dot;

float dt = 0.005;
float Q_angle = 0.001;
float Q_gyro = 0.005;
float R_angle = 0.5;
float C_0 = 1;

_Mpu6050_data Mpu = {0};

float Kalman_Filter(double angle_m, double gyro_m)
{
	angle += (gyro_m - q_bias) * dt;
  angle_err = angle_m - angle;
  Pdot[0] = Q_angle - P[0][1] - P[1][0];
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err; //���ŽǶ�
  q_bias += K_1 * angle_err;
  angle_dot = gyro_m - q_bias; //���Ž��ٶ�
	return angle;
}

float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

float QM_Kp = 100.0f;                        // ��������֧�������������ٶȼ�/��ǿ��
float QM_Ki = 0.2f;                // ��������֧���ʵ�������ƫ�����ν�
float halfT = 0.0025f;                // �������ڵ�һ��

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // ��Ԫ����Ԫ�أ�������Ʒ���
float exInt = 0, eyInt = 0, ezInt = 0;        // ��������С�������
float Yaw,Pitch,Roll;  //ƫ���ǣ������ǣ�������

float QuaternionMethod(float gx, float gy, float gz, float ax, float ay, float az)
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;  

	// ����������
	norm = sqrt(ax*ax + ay*ay + az*az);      
	ax = ax / norm;                   //��λ��
	ay = ay / norm;
	az = az / norm;      

	// ���Ʒ��������
	vx = 2.0f * (q1*q3 - q0*q2);
	vy = 2.0f * (q0*q1 + q2*q3);
	vz = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;

	// ���������ͷ��򴫸��������ο�����֮��Ľ���˻����ܺ�
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);

	// ������������������
	exInt = exInt + ex*QM_Ki;
	eyInt = eyInt + ey*QM_Ki;
	ezInt = ezInt + ez*QM_Ki;

	// ������������ǲ���
	gx = gx + QM_Kp*ex + exInt;
	gy = gy + QM_Kp*ey + eyInt;
	gz = gz + QM_Kp*ez + ezInt;

	// ������Ԫ���ʺ�������
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

	// ��������Ԫ
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;

	//Pitch=asin(2 * q1 * q3 - 2 * q0 * q2)*57.3; // pitch ,ת��Ϊ����

	
	//Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // rollv

	//Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;                //�˴�û�м�ֵ��ע��
	
	Pitch =  atan2(2.0f*(q0*q1 + q2*q3),q0*q0 - q1*q1 - q2*q2 + q3*q3) * rad_to_angle;
	Roll  =  asin(2.0f*(q0*q2 - q1*q3)) * rad_to_angle; 
	
	Yaw += Mpu.deg_s.z * 0.005;
	
	return Yaw;
}

//����butterworth-lpf
float butterworth_lpf(float now_input,_Butterworth_data *buffer, _Butterworth_parameter *parameter)
{
    buffer->input_data[2] = now_input;

    /* Butterworth LPF */
    buffer->output_data[2] =   parameter->b[0] * buffer->input_data[2]
                             + parameter->b[1] * buffer->input_data[1]
                             + parameter->b[2] * buffer->input_data[0]
                             - parameter->a[1] * buffer->output_data[1]
                             - parameter->a[2] * buffer->output_data[0];
    /* x(n) ���� */
    buffer->input_data[0] = buffer->input_data[1];
    buffer->input_data[1] = buffer->input_data[2];
    /* y(n) ���� */
    buffer->output_data[0] = buffer->output_data[1];
    buffer->output_data[1] = buffer->output_data[2];
    
    return buffer->output_data[2];
}

_Butterworth_parameter gyro_30hz_parameter =
{
    //200hz---30hz
    1,  -0.7477891782585,    0.272214937925,
    0.1311064399166,   0.2622128798333,   0.1311064399166 
}; 

_Butterworth_data   gyro_butter_data[3];

//��ȡ����������������
void get_gyro_raw(void)
{
//		gyro_raw.x = gyro_raw_p.x;
//    gyro_raw.y = gyro_raw_p.y;
//    gyro_raw.z = gyro_raw_p.z; 
	
    gyro_raw.x = gyro_raw_p.x - gyro_offset.x;
    gyro_raw.y = gyro_raw_p.y - gyro_offset.y;
    gyro_raw.z = gyro_raw_p.z - gyro_offset.z;        
    
    gyro_raw_f.x = (float)butterworth_lpf(((float)gyro_raw.x),&gyro_butter_data[0],&gyro_30hz_parameter);
    gyro_raw_f.y = (float)butterworth_lpf(((float)gyro_raw.y),&gyro_butter_data[1],&gyro_30hz_parameter);
    gyro_raw_f.z = (float)butterworth_lpf(((float)gyro_raw.z),&gyro_butter_data[2],&gyro_30hz_parameter);
	
//		gyro_raw_f.x = (float)gyro_raw.x;
//    gyro_raw_f.y = (float)gyro_raw.y;
//    gyro_raw_f.z = (float)gyro_raw.z;
}

void get_rad_s(SI_F_XYZ *gyro_in,SI_F_XYZ *gyro_out)
{
	gyro_out->x = (float)(gyro_in->x * gyro_raw_to_radian_s);
	gyro_out->y = (float)(gyro_in->y * gyro_raw_to_radian_s);
	gyro_out->z = (float)(gyro_in->z * gyro_raw_to_radian_s);
}

void get_deg_s(SI_F_XYZ *gyro_in,SI_F_XYZ *gyro_deg_out)
{
	gyro_deg_out->x = (float)(gyro_in->x * gyro_raw_to_deg_s);
	gyro_deg_out->y = (float)(gyro_in->y * gyro_raw_to_deg_s);
	gyro_deg_out->z = (float)(gyro_in->z * gyro_raw_to_deg_s);    
}

//��ȡ���ٶȼ�ԭʼ��������
void get_acc_raw(void)
{
    acc_raw.x = acc_raw_p.x;
    acc_raw.y = acc_raw_p.y;
    acc_raw.z = acc_raw_p.z; 

	//����У׼���������ٶ���
//	acc_raw_f.x = (float)(cal_acc.K[0]*((float)acc_raw.x) - cal_acc.B[0]*one_g_to_acc);
//	acc_raw_f.y = (float)(cal_acc.K[1]*((float)acc_raw.y) - cal_acc.B[1]*one_g_to_acc);
//	acc_raw_f.z = (float)(cal_acc.K[2]*((float)acc_raw.z) - cal_acc.B[2]*one_g_to_acc);
	
	acc_raw_f.x = acc_raw.x;
	acc_raw_f.y = acc_raw.y;
	acc_raw_f.z = acc_raw.z;
}

//��ȡIIR�˲�����
void get_iir_factor(float *out_factor,float Time, float Cut_Off)
{
	*out_factor = Time /( Time + 1/(2.0f * PI * Cut_Off) );
}

//IIR��ͨ�˲���(���ٶ�)
void acc_iir_lpf(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out,float lpf_factor)
{
	acc_out->x = acc_out->x + lpf_factor*(acc_in->x - acc_out->x); 
	acc_out->y = acc_out->y + lpf_factor*(acc_in->y - acc_out->y); 
	acc_out->z = acc_out->z + lpf_factor*(acc_in->z - acc_out->z); 
}

void get_acc_g(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out)
{
	acc_out->x = (float)(acc_in->x * acc_raw_to_g);
	acc_out->y = (float)(acc_in->y * acc_raw_to_g);
	acc_out->z = (float)(acc_in->z * acc_raw_to_g);
}