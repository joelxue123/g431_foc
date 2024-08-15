#ifndef __STM32_MATH_H__
#define __STM32_MATH_H__

typedef struct
{
	int Fiter_in;
	int Fiter_K;
	int Fiter_Out;	
} Fiter;

int Q15_MUL(int Data_A,int Data_B);		//Q15�˷�����  ����Q15,���Q15
int Q16_MUL(int Data_A,int Data_B);		//Q16�˷�����  ����Q16,���Q16
int Q16_Int(long int Data_A);				  //Q16ת����
long int Int_Q16(int Data_A);				  //����תQ16

unsigned int Low_Fiter(Fiter *Ptr);
unsigned int ASM_MUL(unsigned int a,unsigned int b);
unsigned int MUL_DIV(unsigned int Data,unsigned int Q_Data,unsigned int X_Data);	//Data*Q_Data/X_Data
unsigned int Uint32_DIV(unsigned int Data_A,unsigned int Data_B);
unsigned int SQRT_INT(unsigned int Data); //���㿪��
static inline int LIMIT_INT(int data_in,int max,int min)
{
    int rst;
    rst = (data_in >= max) ? max : data_in;
    rst = (rst <= min) ? min : rst;
    return rst;
}

#endif
