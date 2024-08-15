#include "parameter_identification.h"
#include "global.h"

// ʶ������� ����ֵ 4.5�� �����ֵ 80uH�������ֵ��ȥ��ģ�Ϳ���   


static int step =0;
static int  cnt; 
static float R;
static	float _iq;
static	float _v;
	
static int 	curr_index = 0;
static int curr_[1024];

static  enum
{
	IDLE = 0,
	INITIAL,
	WAITTING,
	MEASURE,
	
} measure_curr_step_enum ;

void set_vq(int percent)
{
	g_CmdMap[TAG_OPEN_PWM] = percent;//
}

int get_iq(void)
{
	return g_Iq;
}
static s16 ElectricAngle = 0;
void measure_R(void)
{
	#if 0
	g_ElectricAngle = ElectricAngle;
	
	switch(step)
	{
		case INITIAL:
			ElectricAngle += 1000;
			curr_index = 0;
			cnt = 100000;
			set_vq(30);
			step = WAITTING;
		break;

		case WAITTING:
			
			if( curr_index < 1023)
			{
				curr_[curr_index++] = get_iq();
			}
			
			if(cnt > 0)
				cnt--;
			else
				step = MEASURE;
		break;		
	
		case MEASURE:
			

			
			_iq =get_iq()/1000.0;
			_v = 12*1.732*0.3/3;
			R = _v/_iq;
		
			set_vq(0);
			cnt = 100000;
			step = IDLE;
		
		break;
		
		case IDLE:
			if(cnt > 0)
				cnt--;
			else
				step = INITIAL;

		break;
		
		default:
			
		break;
	}
	
	#endif
	
}