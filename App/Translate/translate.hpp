#pragma once
#include "adc.h"
#include "config.h"
#include "matrix.hpp"

namespace App
{
    class Translate
    {
    public:
        Translate(void);
        void init(void);
        void update(void);
        Matrixf<3, 1> *get_pos(void);
        void resetPos(void);
    private:
             
            float xy_data_[2] = {0.0f, 0.0f};
            uint32_t adc_data_[2] = {0, 0};
            bool key_pressed_ = false;
            Matrixf<3, 1> *pos_; //x,y,z
            uint32_t tick = 0; // 上次更新位置的时间戳
    };
}


