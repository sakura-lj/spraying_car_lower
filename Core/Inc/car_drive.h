#ifndef __car_drive_H__
#define __car_drive_H__

void Car_Init(void);
void carSpeed_set(uint8_t duty);
void spray_set(uint8_t state);
void direction_set(uint8_t direction);
void speed_control(void);
void direction_control(void);
void spray_control(void);

#endif
