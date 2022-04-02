#ifndef __my_klm_lib_H_
#define __my_klm_lib_H_



typedef struct
{
	float x_state;
	float y_observation;
	
	float p_variance;
	float K_gain;
	float R_observe_variance;
	float Q_input_variance;
}first_order_system;



typedef struct
{
	float x_state[2][1];     //״̬
	float y_oberve[2][1];		//�۲�
	float A_matrix[2][2];		//����
	float p_variance[2][2];	//Э����
	float K_gain[2][2]; 		//����������
	float R_variance[2][2]; //�۲�����
	float Q_variance[2][2];	//��������
}Second_order_sys;;



void Sec_Order_sys_Init(Second_order_sys * sys);

/////�ٶ�λ�ö���ϵͳ
void Horizon_klm_estimate(float Pos , float Vel ,Second_order_sys * sys,float delta_T);

void klm_structure_Init(void) ;

extern Second_order_sys Horizon_E_klm,Horizon_N_klm;


extern Second_order_sys   Sys_test1 , Sys_test2 , Sys_test3 ;

#endif

