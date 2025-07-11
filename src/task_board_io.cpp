/* task_board_io.cpp
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

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <Adafruit_MCP23X17.h>
#include "task_board_io.hpp"
#include "task_dmx.hpp"

// Number of spots in queue
static const uint8_t TASKBOARD_IO_QUEUE_SIZE = 10;

// Rate of IO task in milliseconds
static const uint16_t TASKBOARDIO_WAKE_RATE_MS = 1;

static QueueHandle_t TaskBoardIO_queue;

static Adafruit_MCP23X17 board_mcp;

bool TaskBoardIO_setupQueue() {
  TaskBoardIO_queue = xQueueCreate(TASKBOARD_IO_QUEUE_SIZE, sizeof(TaskBoardIO_queue_msg_t));
  if (TaskBoardIO_queue == 0) {
    return false;
  } else {
    return true;
  }
}

BaseType_t TaskBoardIO_mcpPinMode(int pin, PinMode pinMode) {
  TaskBoardIO_queue_msg_t msg = {
    .device = TASKBOARDIO_QUEUE_MSG_DEVICE_BOARD_IO,
    .type = TASKBOARDIO_QUEUE_MSG_TYPE_PINMODE,
    .pin = pin,
    .pinMode = pinMode
  };
  return xQueueSendToBack(TaskBoardIO_queue, (void *)&msg, (TickType_t)0);
}

BaseType_t TaskBoardIO_mcpDigitalWrite(int pin, PinStatus pinStatus) {
  TaskBoardIO_queue_msg_t msg = {
    .device = TASKBOARDIO_QUEUE_MSG_DEVICE_BOARD_IO,
    .type = TASKBOARDIO_QUEUE_MSG_TYPE_DIGITALWRITE,
    .pin = pin,
    .pinStatus = pinStatus
  };
  return xQueueSendToBack(TaskBoardIO_queue, (void *)&msg, (TickType_t)0);
}

void TaskBoardIO(void *pvParameters) {
  (void)pvParameters;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();


  if (!board_mcp.begin_I2C(0x20, &Wire)) {
    Serial1.println("Error.");
    while (1);
  }

  board_mcp.pinMode(12, INPUT_PULLUP);
  board_mcp.pinMode(13, INPUT_PULLUP);
  board_mcp.pinMode(14, INPUT_PULLUP);
  board_mcp.pinMode(15, INPUT_PULLUP);
  board_mcp.pinMode(00, INPUT_PULLUP);
  board_mcp.pinMode(01, INPUT_PULLUP);
  board_mcp.pinMode(02, INPUT_PULLUP);
  board_mcp.pinMode(03, INPUT_PULLUP);
  board_mcp.pinMode(04, INPUT_PULLUP);
  board_mcp.pinMode(05, INPUT_PULLUP);
  board_mcp.pinMode(06, INPUT_PULLUP);
  board_mcp.pinMode(07, INPUT_PULLUP);

  static TaskBoardIO_queue_msg_t currentMsg;
  while (1) {
    while (xQueueReceive(TaskBoardIO_queue, (void *)&currentMsg, (TickType_t)0) == pdTRUE) {
      switch (currentMsg.device) {
        case TASKBOARDIO_QUEUE_MSG_DEVICE_BOARD_IO:
          switch (currentMsg.type) {
            case TASKBOARDIO_QUEUE_MSG_TYPE_PINMODE:
              board_mcp.pinMode(currentMsg.pin, currentMsg.pinMode);
              break;
            case TASKBOARDIO_QUEUE_MSG_TYPE_DIGITALWRITE:
              board_mcp.digitalWrite(currentMsg.pin, currentMsg.pinStatus);
              break;
            default:
              break;
          }
          break;
        default:
          break;
      }
    }

    uint16_t raw_switches = ~(board_mcp.readGPIOAB());

    uint16_t switches = ((raw_switches & 0xF000) >> 12) | ((raw_switches & 0x003F) << 4);

    TaskDMX_setAddress((TaskDMX_address_t)switches);

    vTaskDelayUntil(&xLastWakeTime, (TASKBOARDIO_WAKE_RATE_MS / portTICK_PERIOD_MS));
  }
}
