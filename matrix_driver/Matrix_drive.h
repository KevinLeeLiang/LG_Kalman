#ifndef _Matrix_drive_H__
#define _Matrix_drive_H__




void Matrix_Init(float *pr , int hang , int lie ); 

void Matrix_Init_One (float *matrix , int hang , int lie );

void Matrix_Init_eye (float *matrix , int hang , int lie );

void Matrix_Mul_Cross(float *pre  ,int hang_pre , int lie_pre , float *later,int hang_lat , int lie_lat ,float *res  );

void Matrix_Mul_dot(float *matrix , float mul ,int hang , int lie ,float *res  );

void Matrix_T(float *matrix ,int hang , int lie ,float *res  );

void Matrix_NI(float *matrix_a ,int scale,float *res );

void Matrix_Add(float *matrix_a ,float *matrix_b   ,int hang , int lie ,float *res  );

void Matrix_deduct(float *matrix_a ,float *matrix_b   ,int hang , int lie ,float *res  );


void Matrix_cal_test(void) ;
#endif

