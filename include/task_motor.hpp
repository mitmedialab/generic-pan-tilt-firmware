/* task_motor.hpp
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

#ifndef TASK_MOTOR_H
#define TASK_MOTOR_H

typedef enum s_TaskMotor_motor_addr_t {
  TASKMOTOR_MOTOR_ADDR_0 = 0,
  TASKMOTOR_MOTOR_ADDR_1 = 1,
} TaskMotor_motor_addr_t;

typedef enum s_TaskMotor_mcp_addr_t {
  TASKMOTOR_MOTOR_MCP_0 = 0,
  TASKMOTOR_MOTOR_MCP_1 = 1,
} TaskMotor_mcp_addr_t;

typedef enum s_TaskMotor_motor_msg_type_t {
  TASKMOTOR_MOTOR_MSG_TYPE_GOTO_NORMAL,
  TASKMOTOR_MOTOR_MSG_TYPE_GOTO_SHORTEST,
  TASKMOTOR_MOTOR_MSG_TYPE_GOTO_POS,
  TASKMOTOR_MOTOR_MSG_TYPE_GOTO_NEG,
  TASKMOTOR_MOTOR_MSG_TYPE_VEL_POS,
  TASKMOTOR_MOTOR_MSG_TYPE_VEL_NEG,
  TASKMOTOR_MOTOR_MSG_TYPE_STOP_TILT,
  TASKMOTOR_MOTOR_MSG_TYPE_STOP,
  TASKMOTOR_MOTOR_MSG_TYPE_CLEAR_COMPLETE,
} TaskMotor_motor_msg_type_t;

typedef struct s_TaskMotor_queue_msg_t {
  TaskMotor_motor_msg_type_t type;
  TaskMotor_motor_addr_t motor_addr;
  uint16_t pos;
  uint16_t vel;
} TaskMotor_queue_msg_t;

typedef enum s_TaskMotor_motor_direction_t {
  TASKMOTOR_MOTOR_DIRECTION_POS,
  TASKMOTOR_MOTOR_DIRECTION_NEG,
  TASKMOTOR_MOTOR_DIRECTION_ANY,
  TASKMOTOR_MOTOR_DIRECTION_SHORTEST,
} TaskMotor_motor_direction_t;

bool TaskMotor_setupQueue();
BaseType_t TaskMotor_goto_normal(TaskMotor_motor_addr_t motor, uint16_t pos, uint16_t vel);
BaseType_t TaskMotor_goto_shortest(TaskMotor_motor_addr_t motor, uint16_t pos, uint16_t vel);
BaseType_t TaskMotor_goto_pos(TaskMotor_motor_addr_t motor, uint16_t pos, uint16_t vel);
BaseType_t TaskMotor_goto_neg(TaskMotor_motor_addr_t motor, uint16_t pos, uint16_t vel);
BaseType_t TaskMotor_vel_pos(TaskMotor_motor_addr_t motor, uint16_t vel);
BaseType_t TaskMotor_vel_neg(TaskMotor_motor_addr_t motor, uint16_t vel);
BaseType_t TaskMotor_stop_tilt();
BaseType_t TaskMotor_stop();
void TaskMotor(void *pvParameters) __attribute__ ((flatten));

#endif // TASK_MOTOR_H
