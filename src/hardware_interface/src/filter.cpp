#include "filter.h"
#define FILTER_N 100

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
