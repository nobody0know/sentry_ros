#include "filter.h"
#define FILTER_N 100
//average filter
float Data_filter(float data) {
    int i, j;
    static int count=0;
    float filter_temp=0, filter_sum = 0,filter_out=0;
    static float filter_buf[FILTER_N];
    filter_buf[count] = data;
    count++;
    // 采样值从小到大排列（冒泡法）
    for(j = 0; j < count- 1; j++) {
        for(i = 0; i < count- 1 - j; i++) {
        if(filter_buf[i] > filter_buf[i + 1]) {
            filter_temp = filter_buf[i];
            filter_buf[i] = filter_buf[i + 1];
            filter_buf[i + 1] = filter_temp;
            }
        }
    }
    // 去除最大最小极值后求平均
    for(i = 1; i < count- 1; i++) filter_sum += filter_buf[i];
    filter_out = filter_sum / (count - 2); 
    if (count>=FILTER_N)
    {
        count = 0;
    }
    return filter_out;
}
/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num)
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num = num;
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num / (first_order_filter_type->num + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

//二阶低通滤波
void SetCutoffFreq(second_lowPass_filter *lf,float sample_freq, float cutoff_freq)
{
    float fr =0;
    float ohm =0;
    float c =0;

    fr= sample_freq/cutoff_freq;
    ohm=tan(M_PI/fr);
    c=1.0f+2.0f*cos(M_PI/4.0f)*ohm + ohm*ohm;

    lf->_cutoff_freq1 = cutoff_freq;

    if (lf->_cutoff_freq1 > 0.0f)
    {
        lf->_b01 = ohm*ohm/c;
        lf->_b11 = 2.0f*lf->_b01;
        lf->_b21 = lf->_b01;
        lf->_a11 = 2.0f*(ohm*ohm-1.0f)/c;
        lf->_a21 = (1.0f-2.0f*cosf(M_PI/4.0f)*ohm+ohm*ohm)/c;
    }
}

float Apply(second_lowPass_filter *LF,float sample)
{
    float delay_element_0 = 0, output=0;
    if (LF->_cutoff_freq1 <= 0.0f)
    {
        // no filtering
        return sample;
    }
    else
    {
        delay_element_0 = sample - LF->_delay_element_11 * LF->_a11 - LF->_delay_element_21 * LF->_a21;
        // do the filtering
        if (isnan(delay_element_0) || isinf(delay_element_0)) {
            // don't allow bad values to propogate via the filter
            delay_element_0 = sample;
        }
        output = delay_element_0 * LF->_b01 + LF->_delay_element_11 *LF->_b11 + LF->_delay_element_21 * LF->_b21;

        LF->_delay_element_21 = LF->_delay_element_11;
        LF->_delay_element_11 = delay_element_0;

        // return the value.  Should be no need to check limits
        return output;
    }
}