#ifndef __filter_H
#define __filter_H
#include <math.h>
#include <string.h>
#include "stdint.h"

#define PI 3.1415927f
#define Moving_Len  10
////////////////////////////////////////

//¨°??¡Á?¡§???¨¹??2¡§?a??
typedef struct 
{
	float LastP;
	float Now_P;
	float out;
	float Kg;
	float Q;
	float R;	
}kalman_1_struct;

typedef struct{
	float x;
	float p;
	float A;
	float H;
	float q;
	float r;
	float gain;
}kalman1_state;

extern float kalman_1(kalman_1_struct *ekf,float input);  //¨°????¡§???¨¹
void kalman1_init(kalman1_state *state, float init_x, float init_p);
float kalman1_filter(kalman1_state *state, float z_measure);

///////////////////////////////////////////////////////////////////////////////////
//?D?¦Ì??2¡§
extern int16_t MovMiddle(int16_t input);

////////////////////////////////////////////////////////////////////////

/*  LPF 1st filter   */
typedef struct{
		float old_data;
		float new_data;
		float factor;
}Filter_LPF_1;
extern float LPF_1_Filter_1(Filter_LPF_1 *LPF_1);
extern float LPF_1_Filter_2(Filter_LPF_1 *LPF_1,float dt);


typedef struct {
	
	uint16_t cnt;
	int16_t input;
	int16_t *average;

	uint8_t max_cnt;
}MovAverage;


int16_t AntiPulse_MovingAverage_Filter(MovAverage *_MovAverage);



short Moving_filter(short data);
///////////////////////////////////////////////////////////////////////////////////
#endif
