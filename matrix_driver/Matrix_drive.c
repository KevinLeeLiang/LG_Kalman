#include "Matrix_drive.h"
#include <stdio.h>
#include "Headfile.h"


void Matrix_Init(float *matrix , int hang , int lie )
{
	int i=0;
	int j=0 ;
	int a=0;
	for(i=0 ; i< hang ;i++ )
	{
		
		for( j=0; j< lie ; j++ )
		{
		  matrix[ i*lie + j ] =  a; 
			a++ ;
		}
	}
}

void Matrix_Init_One (float *matrix , int hang , int lie )
{
	int i=0;
	int j=0 ;
	for(i=0 ; i< hang ;i++ )
	{
		
		for( j=0; j< lie ; j++ )
		{
		  matrix[ i*lie + j ] =  1 ; 
		}
	}
}

void Matrix_Init_eye (float *matrix , int hang , int lie )
{
	int i=0;
	int j=0 ;
	
	if(hang == lie)
	{
		for(i=0 ; i< hang ;i++ )
		{
			
			for( j=0; j< lie ; j++ )
			{
				if( j == i ) 
				{
					matrix[ i*lie + j ] =  1 ; 
				}
				else
				{
					matrix[ i*lie + j ] =  0 ;
				}
				
			}
		}
	}
}

//
//¾ØÕó²æ³Ëº¯Êý£º
//ÊäÈë²ÎÊý£º                        
//    ×ó¾ØÕóµØÖ· £¬ ×ó¾ØÕóÐÐ£¬×ó¾ØÕóÁÐ
//    ÓÒ¾ØÕóµØÖ· £¬ ÓÒ¾ØÕóÐÐ£¬ÓÒ¾ØÕóÁÐ		
//		½á¹û¾ØÕóµØÖ·
//
void Matrix_Mul_Cross(float *pre  ,int hang_pre , int lie_pre , float *later,int hang_lat , int lie_lat ,float *res  )
{	
	int i_pr=0;
	int j_pr=0 ;
	int i=0;
	float hang_total =0;
	if( lie_pre == hang_lat )
	{
		for(i_pr=0 ; i_pr< hang_pre ;i_pr++ )
		{
			for( j_pr=0; j_pr< lie_lat ; j_pr ++ )
			{
				hang_total=0;
				for(i=0 ; i<lie_pre ; i++)
				{
					hang_total += pre[ i_pr*lie_pre + i ] * later[ j_pr + i*lie_lat ] ;
				}
				res[ i_pr*lie_lat + j_pr ] = hang_total ;
			}
		}
	}
}



void Matrix_Mul_dot(float *matrix , float mul ,int hang , int lie ,float *res  )
{	
	int i=0,j=0;
	for(i=0 ; i< hang ;i++ )
	{
		for( j=0; j<lie ; j++ )
		{
				res[ i*lie + j ] =  matrix[ i*lie + j ] * mul ;
		}
	}
}

void Matrix_T(float *matrix ,int hang , int lie ,float *res  )
{	
	int i=0,j=0;
	for(i=0 ; i< hang ;i++ )
	{
		for( j=0; j<lie ; j++ )
		{
				res[ i + j*hang ] =  matrix[ i*lie + j ]  ;
		}
	}
}


void Matrix_Add(float *matrix_a ,float *matrix_b   ,int hang , int lie ,float *res  )
{
	int i=0,j=0;
	
	for(i=0 ; i< hang ;i++ )
	{
		for( j=0; j<lie ; j++ )
		{
				res[ i*lie + j ] =  matrix_a[i*lie + j] +  matrix_b[i*lie + j] ;
		}
	}

}


void Matrix_deduct(float *matrix_a ,float *matrix_b   ,int hang , int lie ,float *res  )
{
	int i=0,j=0;
	
	for(i=0 ; i< hang ;i++ )
	{
		for( j=0; j<lie ; j++ )
		{
				res[ i*lie + j ] =  matrix_a[ i*lie + j] -  matrix_b[  i*lie + j ] ;
		}
	}

}


void Matrix_NI(float *matrix_a ,int scale,float *res )
{
	float A_value=0;
	if(scale==2)
	{
		A_value = matrix_a[ 0 ] * matrix_a[2+1] - matrix_a[ 1 ] * matrix_a[2] ;
		if(A_value!= 0 )
		{
			res[0] = matrix_a[3] /A_value ;
			res[3] = matrix_a[0] /A_value ;
			res[1] = -matrix_a[2] /A_value ;
			res[2] = -matrix_a[1] /A_value ;
		}
	}
}




											
void Matrix_cal_test()
{
//	
//  //	Matrix_Init_eye( (float *)&Matrix_x , 5 ,5 ) ;
//	
//  Matrix_Mul_Cross((float *)&Matrix_1 ,3,4 ,(float *)&Matrix_2 ,4,2 ,(float *)&Matrix_3);
//
//	Matrix_T( (float *)&Matrix_2 ,4,2 , (float *)&Matrix_4 ) ;
//	
	
	
	
	
}
