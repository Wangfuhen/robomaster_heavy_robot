#ifndef CAN_INIT_H
#define CAN_INIT_H
#include "main.h"
#include "c_data_typedef.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern void can_filter_init(void);

#endif
