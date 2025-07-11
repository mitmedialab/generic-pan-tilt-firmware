/* task_motor_reg_tmc5240.hpp
 *
 * Created by Perry Naseck on 2024-08-29.
 *
 * This file is a part of the Generic Pan Tilt Firmware
 * https://github.com/mitmedialab/generic-pan-tilt-firmware
 *
 * Copyright (c) 2024-2025, MIT Media Lab
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TASK_MOTOR_REG_TMC5240_H
#define TASK_MOTOR_REG_TMC5240_H

#define BIT(n) (1UL << (n))

#define DEFAULT_CUR_HOLD 10 // out of 31
#define DEFAULT_CUR_RUN  7  // out of 31

#define DEFAULT_ACCEL 4000
#define DEFAULT_VEL   100000
#define DEFAULT_D1_D2 0xa

#define TMC5240_NREG      (sizeof(TMC5240_ALL_REGS))
#define TMC5240_REG_MIN   TMC5240_REG_GSTAT
#define TMC5240_REG_MAX   TMC5240_REG_SG4_IND
#define TMC5240_REG_RANGE (TMC5240_REG_MAX - TMC5240_REG_MIN)
#define TMC5240_STATUS_STOP_R      BIT(7)
#define TMC5240_STATUS_STOP_L      BIT(6)
#define TMC5240_STATUS_POS_REACHED BIT(5)
#define TMC5240_STATUS_VEL_REACHED BIT(4)
#define TMC5240_STATUS_STANDSTILL  BIT(3)
#define TMC5240_STATUS_SG2         BIT(2)
#define TMC5240_STATUS_DRV_ERR     BIT(1)
#define TMC5240_STATUS_RESET       BIT(0)

#define TMC5240_CHOPCONF_INTPOL         BIT(28)
#define TMC5240_CHOPCONF_TBL_DEFAULT    (0b10 << 15)
#define TMC5240_CHOPCONF_HSTART_DEFAULT (0x2 << 7)
#define TMC5240_CHOPCONF_HEND_DEFAULT   (0x5 << 4)
#define TMC5240_CHOPCONF_DEFAULT \
	(TMC5240_CHOPCONF_INTPOL | TMC5240_CHOPCONF_TBL_DEFAULT | \
	 TMC5240_CHOPCONF_HSTART_DEFAULT | TMC5240_CHOPCONF_HEND_DEFAULT)
#define TMC5240_CHOPCONF_DRVEN_MASK 0xf
#define TMC5240_CHOPCONF_DRVEN      (TMC5240_CHOPCONF_DEFAULT | 0b0010)

#define TMC5240_SW_MODE_DEFAULT 0x0
#define TMC5240_RAMP_STAT_CLEAR 0xff

#define TMC5240_RAMP_STAT_VSTOP_R         BIT(15)
#define TMC5240_RAMP_STAT_VSTOP_L         BIT(14)
#define TMC5240_RAMP_STAT_SG              BIT(13)
#define TMC5240_RAMP_STAT_SECOND_MOVE     BIT(12)
#define TMC5240_RAMP_STAT_T_ZEROWAIT      BIT(11)
#define TMC5240_RAMP_STAT_VZERO           BIT(10)
#define TMC5240_RAMP_STAT_POS_REACHED     BIT(9)
#define TMC5240_RAMP_STAT_VEL_REACHED     BIT(8)
#define TMC5240_RAMP_STAT_EVT_POS_REACHED BIT(7)
#define TMC5240_RAMP_STAT_EVT_STOP_SG     BIT(6)
#define TMC5240_RAMP_STAT_EVT_STOP_R      BIT(5)
#define TMC5240_RAMP_STAT_EVT_STOP_L      BIT(4)
#define TMC5240_RAMP_STAT_LATCH_R         BIT(3)
#define TMC5240_RAMP_STAT_LATCH_L         BIT(2)
#define TMC5240_RAMP_STAT_STOP_R          BIT(1)
#define TMC5240_RAMP_STAT_STOP_L          BIT(0)

#define TMC5240_IOIN_ADC_ERR     BIT(15)
#define TMC5240_IOIN_DRV_ENN     BIT(4)
#define TMC5240_IOIN_EXT_RES_DET BIT(3)

#define TMC5240_DRV_STATUS_OLB   BIT(30)
#define TMC5240_DRV_STATUS_OLA   BIT(29)
#define TMC5240_DRV_STATUS_S2GB  BIT(28)
#define TMC5240_DRV_STATUS_S2GA  BIT(27)
#define TMC5240_DRV_STATUS_OTPW  BIT(26)
#define TMC5240_DRV_STATUS_OT    BIT(25)
#define TMC5240_DRV_STATUS_SG    BIT(24)
#define TMC5240_DRV_STATUS_S2VSB BIT(13)
#define TMC5240_DRV_STATUS_S2VSA BIT(12)

#endif // TASK_MOTOR_REG_TMC5240_H
