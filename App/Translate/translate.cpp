#include "translate.hpp"

namespace App
{
    Translate::Translate(void)
    {
        pos_ = new Matrixf<3, 1>();
    }

    void Translate::init(void)
    {
        if (HAL_ADC_Start_DMA(&ADC_PORT, adc_data_, 2) != HAL_OK)
        {
            Error_Handler();
        }
    }

    void Translate::update(void)
    {
        //分正负量程
        float dx = (float)adc_data_[0] - TRASLATE_ADC_MID_X;
        xy_data_[0] = dx >= 0 ? dx * TRASLATE_ADC_SCALE_X_POS : dx * TRASLATE_ADC_SCALE_X_NEG;
        float dy = (float)adc_data_[1] - TRASLATE_ADC_MID_Y;
        xy_data_[1] = dy >= 0 ? dy * TRASLATE_ADC_SCALE_Y_POS : dy * TRASLATE_ADC_SCALE_Y_NEG;

        key_pressed_ = HAL_GPIO_ReadPin(key_z_GPIO_Port, key_z_Pin);

        //位置矩阵增量


        // 更新位置矩阵
        (*pos_)[0][0] = xy_data_[0]; // x坐标
        (*pos_)[1][0] = xy_data_[1]; // y坐标
        (*pos_)[2][0] = 0.0f;         // z坐标（假设为0）

        
    }

    Matrixf<3, 1> *Translate::get_pos(void)
    {
        return pos_;
    }
}