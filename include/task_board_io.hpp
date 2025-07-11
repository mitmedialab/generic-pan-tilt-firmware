/* task_board_io.hpp
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

#ifndef TASK_BOARD_IO_H
#define TASK_BOARD_IO_H

typedef enum s_TaskBoardIO_queue_msg_type_t {
  TASKBOARDIO_QUEUE_MSG_TYPE_PINMODE,
  TASKBOARDIO_QUEUE_MSG_TYPE_DIGITALWRITE,
} TaskBoardIO_queue_msg_type_t;

typedef enum s_TaskBoardIO_queue_msg_device_t {
  TASKBOARDIO_QUEUE_MSG_DEVICE_BOARD_IO,
} TaskBoardIO_queue_msg_device_t;

typedef struct s_TaskBoardIO_queue_msg_t {
  TaskBoardIO_queue_msg_device_t device;
  TaskBoardIO_queue_msg_type_t type;
  int pin;
  union {
    PinMode pinMode;
    PinStatus pinStatus;
  };
} TaskBoardIO_queue_msg_t;

bool TaskBoardIO_setupQueue();
BaseType_t TaskBoardIO_mcpPinMode(int pin, PinMode mode);
BaseType_t TaskBoardIO_mcpDigitalWrite(int pin, PinStatus state);
void TaskBoardIO(void *pvParameters) __attribute__ ((flatten));

#endif // TASK_BOARD_IO_H
