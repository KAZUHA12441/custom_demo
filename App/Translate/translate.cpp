#include "translate.hpp"

namespace App
{
    Translate::Translate(void)
    {
        pos_ = new Matrixf<3, 1>();
        (*pos_).arm_mat_.pData[0] = 0.0f; // x坐标
        (*pos_).arm_mat_.pData[1] = 0.0f; // y坐标
        (*pos_).arm_mat_.pData[2] = 0.0f; // z坐标
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

        // 低通滤波，抑制高频噪声尖峰
        static float xy_f[2] = {0.0f, 0.0f};
        constexpr float alpha = 0.05f;
        xy_f[0] += alpha * (adc_data_[0] - xy_f[0]);
        xy_f[1] += alpha * (adc_data_[1] - xy_f[1]);

        // 分正负量程
        float dx = (float)xy_f[0] - TRASLATE_ADC_MID_X;
        xy_data_[0] = dx >= 0 ? dx * TRASLATE_ADC_SCALE_X_POS : dx * TRASLATE_ADC_SCALE_X_NEG;

        float dy = (float)xy_f[1] - TRASLATE_ADC_MID_Y;
        xy_data_[1] = dy >= 0 ? dy * TRASLATE_ADC_SCALE_Y_POS : dy * TRASLATE_ADC_SCALE_Y_NEG;

        key_pressed_ = HAL_GPIO_ReadPin(key_z_GPIO_Port, key_z_Pin);

        // 更新位置矩阵
        // 只在xy平面上做增量更新，z坐标保持不变

        if (xy_data_[0] > -20.0f && xy_data_[0] < 20.0f)
            xy_data_[0] = 0.0f;
        else if (xy_data_[0] > 100.0f)
            (*pos_).arm_mat_.pData[1] = -0.001f;
        else if (xy_data_[0] < -100.0f)
            (*pos_).arm_mat_.pData[1] = 0.001f;

        if (xy_data_[1] > -20.0f && xy_data_[1] < 20.0f)
            xy_data_[1] = 0.0f;
        else if (xy_data_[1] > 100.0f)
            (*pos_).arm_mat_.pData[0] = -0.001f;
        else if (xy_data_[1] < -100.0f)
            (*pos_).arm_mat_.pData[0] = 0.001f;

        if (xy_data_[0] == 0.0f && xy_data_[1] == 0.0f)
        {
            resetPos();
            tick = 0;
        }
        
        
    }

    void Translate::resetPos(void)
    {
        (*pos_).arm_mat_.pData[0] = 0.0f;
        (*pos_).arm_mat_.pData[1] = 0.0f;
        (*pos_).arm_mat_.pData[2] = 0.0f;
    }

    Matrixf<3, 1> *Translate::get_pos(void)
    {
        return pos_;
    }
}