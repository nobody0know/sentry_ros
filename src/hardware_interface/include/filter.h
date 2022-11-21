#ifndef DATA_FILTER
#define DATA_FILTER
#include <math.h>
float Data_filter(float data);

typedef struct
{
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num;       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
}first_order_filter_type_t;
//一阶滤波初始化
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num);
//一阶滤波计算
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
//二阶低通滤波
//低通
typedef struct {

    float           _cutoff_freq1;
    float           _a11;
    float           _a21;
    float           _b01;
    float           _b11;
    float           _b21;
    float           _delay_element_11;        // buffered sample -1
    float           _delay_element_21;        // buffered sample -2
}second_lowPass_filter;
void SetCutoffFreq(second_lowPass_filter *lf,float sample_freq, float cutoff_freq);
float Apply(second_lowPass_filter *LF,float sample);

#endif