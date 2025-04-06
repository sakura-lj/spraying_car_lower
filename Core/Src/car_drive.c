#include "dac.h"
#include "OLED.h"
#include "Remote_control.h"
#include "gpio.h"
#include "car_drive.h"
extern volatile uint8_t is_open;
extern volatile uint8_t CH3, CH4, CH5, CH6, CH7;
volatile uint8_t direction_state = 0, spray_state = 0; // 0: 停止, 1: 前进, 2: 后退

void Car_Init(void) {
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
}
/**
  * @brief  设置电机速度
  * @param  speed: 电机速度，范围0-102
  * @retval 串口可用
  */
void carSpeed_set(uint8_t duty) {   // 控制器的电压范围0-4.3V，电机的电压范围1.2-3.8V
    double temp;
    if(duty == 1){temp = 0;}
    else if(duty >= 2 && duty <= 101){temp = 100 + (duty - 2) * 2;}
    else if(duty == 102){temp = 310;}
    else{temp = 0;} // 默认值，防止意外情况
    temp /= 100;  
    temp = temp * 4096 / 3.3;
    if (temp > 4096) {
        temp = 0;
    }
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)temp);
}

void spray_set(uint8_t state) {
    if (state == 1) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET); // 打开喷雾
    } else {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET); // 关闭喷雾
    }
}
// 设置方向函数,串口可用
void direction_set(uint8_t direction) {
    if (direction == 0) {
        HAL_GPIO_WritePin(backward_GPIO_Port, backward_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(forward_GPIO_Port, forward_Pin, GPIO_PIN_RESET);
    } else if (direction == 1) {
        HAL_GPIO_WritePin(backward_GPIO_Port, backward_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(forward_GPIO_Port, forward_Pin, GPIO_PIN_RESET);
    } else if (direction == 2) {
        HAL_GPIO_WritePin(backward_GPIO_Port, backward_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(forward_GPIO_Port, forward_Pin, GPIO_PIN_SET);
    }
}

void speed_control(void) {
    static uint8_t prev_CH3 = 0;
    uint16_t speed = 0;
    if (is_open == 1){
        CH3 = CH3_GetDuty();
        if (direction_state == 0) {carSpeed_set(0);} 
        else if (CH3 != prev_CH3) {
            carSpeed_set(CH3);
            OLED_Printf(64,0,OLED_6X8,"speed:%04d",speed);
            OLED_Update();
            prev_CH3 = CH3; // 更新上一次的CH3值
        }
    }else{
        carSpeed_set(0); // 关闭电机
    }
}

void direction_control(void) {
    static uint8_t prev_CH5 = 0;
    if (is_open == 1) {
        CH5 = CH5_GetDuty();
        if (CH5 != prev_CH5) {
            if (CH5 == 51) {
                direction_state = 0; // 更新为停止状态
            } else if (CH5 == 86 && direction_state == 0) {
                direction_state = 1; // 更新为前进状态
            } else if (CH5 == 19 && direction_state == 0) {
                //后退状态 PC13设置低电平 PC14设置高电平
                direction_state = 2; // 更新为后退状态
            }
            prev_CH5 = CH5; // 更新上一次的CH5值
            direction_set(direction_state); // 设置方向
        }
    }else{
        direction_set(0); // 关闭方向
    }
}

void spray_control(void) {
    static uint8_t prev_CH6 = 0;
    if (is_open == 1) {
        CH6 = CH6_GetDuty();
        if (CH6 != prev_CH6) {
            if (CH6 >= 10) {
                spray_state = 1; // 更新为喷雾状态
                spray_set(spray_state); // 打开喷雾
            } else {
                spray_state = 0; // 更新为停止喷雾状态
                spray_set(spray_state); // 关闭喷雾
            }
            prev_CH6 = CH6;
        }
    }else{
        spray_set(0); // 关闭喷雾
    }
}

