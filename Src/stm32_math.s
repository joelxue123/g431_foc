.syntax unified
.cpu cortex-m4
.thumb

.section .text

.global Q16_Int
.global Int_Q16
.global Q16_MUL
.global Q15_MUL
.global Q15_PARK
.global Float_Mul
.global Float_DIV
.global MUL_DIV
.global ASM_MUL
.global Uint32_DIV
.global Low_Filter
.global SQRT_INT

Q16_Int:
		add		r0,#0x8000		
		asr		r0,#16			  
		bx		r14


Int_Q16:
		lsl		r0,#16			  
		bx		r14

Q16_MUL:
		smull	r0,r1,r0,r1		
		lsr		r0,#16			  
		add		r0,r0,r1,lsl #16
		bx		r14

Q15_MUL:
		smull	r0,r1,r0,r1		
		lsr		r0,#15			 
		add		r0,r0,r1,lsl #17
		bx		r14

Q15_PARK:
		push	{r4,r5}
		pop		{r4,r5}
		bx		r14



SQRT_INT:	
			//;��ֵΪA�Ľ��ƿ���,r0,Ҫ����������
			movs	r0,r0
			bne		SQRT_INT1
			bx		r14					//  ;Ϊ��ֱ�ӷ���
      
SQRT_INT1:
			mov		r1,#32
			mov		r2,#0x80000000
      
SQRT_INT2:
			tst		r0,r2
			sub		r1,#1
			lsr		r2,#1
			beq		SQRT_INT2			
      
SQRT_INT3:
			lsr		r1,#1				//  ;����/2ȡ��
			mov		r2,#1
			lsl		r1,r2,r1			//;��ֵ

	
			sdiv	r2,r0,r1		//	;A/Xn
			sub		r2,r1			//	  ;A/Xn - Xn
			asr		r2,#1			//	  ;/2
			add		r1,r2			//	  ;Xn+1
			
			//;�ڶ��ε���
			sdiv	r2,r0,r1		//	;A/Xn
			sub		r2,r1			//	  ;A/Xn - Xn
			asr		r2,#1			//	  ;/2
			add		r1,r2			//	  ;Xn+1

			//;�����ε���
			sdiv	r2,r0,r1		//	;A/Xn
			sub		r2,r1			//	  ;A/Xn - Xn
			asr		r2,#1			//	  ;/2
			add		r1,r2			//	  ;Xn+1

			//;���Ĵε���
			sdiv	r2,r0,r1		//	;A/Xn
			sub		r2,r1			//	  ;A/Xn - Xn
			asr		r2,#1			//	  ;/2
			add		r0,r1,r2		 // ;Xn+1

			;mov		r0,r1
			bx		r14



//=============================================================
// ----Function: unsigned int MUL_DIV(unsigned int Data,
//					unsigned int Q_Data,unsigned int X_Data);
// -Description: Data*Q_Data/X_Data
// --Parameters: ������Ҫ�������ݸ�ʽת��
// -----Returns: 
// -------Notes: 
//============================================================= 
MUL_DIV:
			mul		r3,r0,r1 	//	;r0*r1 --> r3
			sdiv	r0,r3,r2 	//	;r3/r2 --> r0
			bx    r14

//=============================================================
// ----Function: unsigned int ASM_MUL(unsigned int Data_A,unsigned int Data_B);
// -Description: Data_A*Data_B
// --Parameters: ������Ҫ�������ݸ�ʽת��
// -----Returns: 
// -------Notes: 
//=============================================================      
ASM_MUL:
			mul		r0,r0,r1 		//r0*r1 --> r0
			bx    r14
            
//;==============================================================================================
////; ----Function: unsigned int 32_16_DIV(unsigned int Data_A,unsigned int Data_B);
//; -Description: 32λ��32λ����,��32λ���
//; --Parameters: ������/����
//; -----Returns: ���,�޷�����
//; -------Notes: 
//;==============================================================================================
Uint32_DIV:
      udiv	r0,r1 		
			bx    r14

Int_Q30:
			mul		r0,r1
			bx		r14

