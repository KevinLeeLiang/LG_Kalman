#include "my_klm_lib.h"
#include "Headfile.h"
#include "Matrix_drive.h"

Second_order_sys Horizon_E_klm,Horizon_N_klm;

 Second_order_sys   Sys_test1 , Sys_test2 , Sys_test3 ;


void Sec_Order_sys_Init(Second_order_sys * sys)
{	
	Matrix_Init_eye((float *)&(sys->A_matrix) ,2,2);
	//sys->A_matrix[0][1] = 1;       ////////////////////////
	Matrix_Init_eye((float *)&(sys->p_variance) ,2,2);
	//Matrix_Mul_dot((float *)&(sys->p_variance),100 ,2,2,(float *)&(sys->p_variance));
	
	sys->R_variance[0][0] = 0.01;
	sys->R_variance[1][1] = 0.01;
	
	sys->Q_variance[0][0]=0.001;
	sys->Q_variance[1][1]=0.001;
	
}




/////速度位置二阶系统
void Horizon_klm_estimate(float Pos , float Vel ,Second_order_sys * sys,float delta_T)
{
	sys->A_matrix[0][1] = delta_T;
	
	sys->y_oberve[0][0] = Pos ;
	sys->y_oberve[1][0] = Vel ;
	
	sys->x_state[0][0]= sys->x_state[0][0] + delta_T * sys->x_state[1][0] ;/////////////step1 predict x

///////////////////////////////////////////////////////////////////////////step2 predict P
	float  P_temp[2][2] ,P_temp2[2][2]; 
	float  A_T_Matrix[2][2] ;
	Matrix_Mul_Cross((float *)(&sys->A_matrix) , 2,2 , (float *)&(sys->p_variance) , 2 , 2 ,(float *)&P_temp) ;
	Matrix_T((float *)&(sys->A_matrix),2,2,(float *)&A_T_Matrix);
	Matrix_Mul_Cross((float *)&P_temp , 2,2 , (float *)&A_T_Matrix , 2 , 2 ,(float *)&P_temp2) ;
	Matrix_Add((float *)&P_temp2 , (float *)&(sys->Q_variance) ,2,2, (float *)&(sys->p_variance)) ;
	
///////////////////////////////////////////////////////////////////////////step3 calculate K
	float matrix3_NI[2][2] ;
	float K_temp[2][2];
	Matrix_Add((float *)&(sys->p_variance) , (float *)&(sys->R_variance) ,2,2, (float *)&K_temp) ;
	Matrix_NI((float *)&K_temp,2,(float *)&matrix3_NI) ;
	Matrix_Mul_Cross((float *)&(sys->p_variance),2,2,(float *)&matrix3_NI ,2,2,(float *)&(sys->K_gain));
	
///////////////////////////////////////////////////////////////////////////step4 correct x
	float Z_delta[2][1] ;
	float statue_correct[2][1] ;
	
	Z_delta[0][0] = sys->x_state[0][0] - sys->y_oberve[0][0];
	Z_delta[1][0] = sys->x_state[1][0] - sys->y_oberve[1][0];
	
	Matrix_Mul_Cross((float *)&(sys->K_gain),2,2,(float *)&Z_delta ,2,1,(float *)&statue_correct);
	
	sys->x_state[0][0] =  sys->x_state[0][0] - statue_correct[0][0];
  sys->x_state[1][0] =  sys->x_state[1][0] - statue_correct[1][0];
	
///////////////////////////////////////////////////////////////////////////	step5 update P
	float P_update[2][2] ;
	
	Matrix_Init_eye((float *)&P_temp ,2,2);
	Matrix_deduct((float *)&P_temp, (float *)&(sys->K_gain) ,2,2,(float *)&P_temp);
	Matrix_Mul_Cross((float *)&P_temp,2,2,(float *)&(sys->p_variance) ,2,2,(float *)&P_update);
	Matrix_Mul_dot((float *)&P_update,1,2,2,(float *)&(sys->p_variance));
	
}



void klm_structure_Init()
{
	Sec_Order_sys_Init(&Horizon_E_klm);
	Sec_Order_sys_Init(&Horizon_N_klm);

}

