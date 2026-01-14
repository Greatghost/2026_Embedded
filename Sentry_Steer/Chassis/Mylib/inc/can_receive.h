#ifndef _CAN_RECEIVE_H
#define _CAN_RECEIVE_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "debug.h"
#include "can_config.h"
#include "GimbalReceive.h"

#include "ChasisController.h"
#include "NingCap.h"

/*  Task */

void CanReceiveAll(CAN_TypeDef *can, CanRxMsg *rx_message);

#endif // !_CAN1_RECEIVE_H
