/* task_dmx.cpp
 *
 * Created by Perry Naseck on 2024-09-01.
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
#include <DmxInput.h>
#include "task_dmx.hpp"
#include "task_motor.hpp"

// Rate of DMX task in milliseconds
static const uint16_t TASKDMX_WAKE_RATE_MS = 5;

static const uint16_t START_CHANNEL_MIN = 1;
static const uint16_t NUM_CHANNELS = 7;
static const uint8_t DMX_RX_PIN = 9;
static const uint8_t DMX_RE_PIN = 10;
static const uint8_t DMX_DE_PIN = 11;
static const uint8_t DMX_START_CODE = 0;
static const uint16_t DMX_ADDRESS_ALL_MAX = 512;

typedef enum s_TaskDMX_dmx_pan_mode_t {
  TASKDMX_DMX_PAN_MODE_POS_NORMAL = 1 * 36, // less than this val is in mode
  TASKDMX_DMX_PAN_MODE_POS_SHORTEST = 2 * 36,
  TASKDMX_DMX_PAN_MODE_POS_POS = 3 * 36,
  TASKDMX_DMX_PAN_MODE_POS_NEG = 4 * 36,
  TASKDMX_DMX_PAN_MODE_VEL_POS = 5 * 36,
  TASKDMX_DMX_PAN_MODE_VEL_NEG = 6 * 36,
  TASKDMX_DMX_PAN_MODE_STOP_TILT = UINT8_MAX - 1, // leq
  TASKDMX_DMX_PAN_MODE_STOP = UINT8_MAX
} TaskDMX_dmx_pan_mode_t;

class DmxInputWait : public DmxInput {
  public:
    void readWait(volatile uint8_t *buffer);
  private:
};

void DmxInputWait::readWait(volatile uint8_t *buffer) { 
  if (_buf == nullptr) {
    read_async(buffer);
  }
  unsigned long start = _last_packet_timestamp;
  while (_last_packet_timestamp == start) {
    // tight_loop_contents();
    vTaskDelay(TASKDMX_WAKE_RATE_MS / portTICK_PERIOD_MS);
  }
}

DmxInputWait dmxInput;

volatile uint8_t dmxBuffer[DMXINPUT_BUFFER_SIZE(START_CHANNEL_MIN, DMX_ADDRESS_ALL_MAX)];

static QueueHandle_t TaskDMX_queue;

bool TaskDMX_setupQueue() {
  // Queue size is 1 so that most current address is always used
  // This requires the use of xQueueOverwrite()
  const uint8_t QUEUE_SIZE = 1;
  TaskDMX_queue = xQueueCreate(QUEUE_SIZE, sizeof(TaskDMX_address_t));
  if (TaskDMX_queue == 0) {
    return false;
  } else {
    return true;
  }
}

void TaskDMX_setAddress(TaskDMX_address_t address) {
  xQueueOverwrite(TaskDMX_queue, (void *)&address);
}

void TaskDMX(void *pvParameters) {
  (void)pvParameters;

  pinMode(DMX_RE_PIN, OUTPUT);
  pinMode(DMX_DE_PIN, OUTPUT);
  digitalWrite(DMX_RE_PIN, LOW);
  digitalWrite(DMX_DE_PIN, LOW);
  taskENTER_CRITICAL();
  dmxInput.begin(DMX_RX_PIN, START_CHANNEL_MIN, DMX_ADDRESS_ALL_MAX);
  taskEXIT_CRITICAL();

  static TaskDMX_address_t dmx_start = START_CHANNEL_MIN;
  static TaskDMX_address_t dmx_start_switches = START_CHANNEL_MIN;
  while (1) {
    if (xQueueReceive(TaskDMX_queue, (void *)&dmx_start_switches, (TickType_t)0) == pdTRUE) {
      if (dmx_start_switches != dmx_start) {
        Serial1.print("new address ");
        Serial1.println(dmx_start_switches);
        dmx_start = dmx_start_switches;
      }
    }
    if (dmx_start >= START_CHANNEL_MIN && dmx_start <= (DMX_ADDRESS_ALL_MAX - NUM_CHANNELS) + 1) {

      dmxInput.readWait(dmxBuffer);
      if (dmxBuffer[0] == DMX_START_CODE) {
        uint16_t x_in = ((uint16_t)dmxBuffer[dmx_start] << 8) | dmxBuffer[dmx_start+1];
        uint16_t y_in = ((uint16_t)dmxBuffer[dmx_start+2] << 8) | dmxBuffer[dmx_start+3];
        uint8_t dmx_pan_mode_raw = dmxBuffer[dmx_start+4];
        uint16_t x_vel_in = ((uint16_t)dmxBuffer[dmx_start+5] << 8) | dmxBuffer[dmx_start+6];

        TaskDMX_dmx_pan_mode_t pan_mode = TASKDMX_DMX_PAN_MODE_POS_NORMAL;
        if (dmx_pan_mode_raw < (uint8_t)TASKDMX_DMX_PAN_MODE_POS_NORMAL) {
          pan_mode = TASKDMX_DMX_PAN_MODE_POS_NORMAL;
        } else if (dmx_pan_mode_raw < (uint8_t)TASKDMX_DMX_PAN_MODE_POS_SHORTEST) {
          pan_mode = TASKDMX_DMX_PAN_MODE_POS_SHORTEST;
        } else if (dmx_pan_mode_raw < (uint8_t)TASKDMX_DMX_PAN_MODE_POS_POS) {
          pan_mode = TASKDMX_DMX_PAN_MODE_POS_POS;
        } else if (dmx_pan_mode_raw < (uint8_t)TASKDMX_DMX_PAN_MODE_POS_NEG) {
          pan_mode = TASKDMX_DMX_PAN_MODE_POS_NEG;
        } else if (dmx_pan_mode_raw < (uint8_t)TASKDMX_DMX_PAN_MODE_VEL_POS) {
          pan_mode = TASKDMX_DMX_PAN_MODE_VEL_POS;
        } else if (dmx_pan_mode_raw < (uint8_t)TASKDMX_DMX_PAN_MODE_VEL_NEG) {
          pan_mode = TASKDMX_DMX_PAN_MODE_VEL_NEG;
        } else if (dmx_pan_mode_raw <= (uint8_t)TASKDMX_DMX_PAN_MODE_STOP_TILT) {
          pan_mode = TASKDMX_DMX_PAN_MODE_STOP_TILT;
        } else { // Anything else is STOP
          pan_mode = TASKDMX_DMX_PAN_MODE_STOP;
        }

        switch (pan_mode) {
          case TASKDMX_DMX_PAN_MODE_POS_NORMAL:
            TaskMotor_goto_normal(TASKMOTOR_MOTOR_ADDR_0, x_in, x_vel_in);
            break;
          case TASKDMX_DMX_PAN_MODE_POS_SHORTEST:
            TaskMotor_goto_shortest(TASKMOTOR_MOTOR_ADDR_0, x_in, x_vel_in);
            break;
          case TASKDMX_DMX_PAN_MODE_POS_POS:
            TaskMotor_goto_pos(TASKMOTOR_MOTOR_ADDR_0, x_in, x_vel_in);
            break;
          case TASKDMX_DMX_PAN_MODE_POS_NEG:
            TaskMotor_goto_neg(TASKMOTOR_MOTOR_ADDR_0, x_in, x_vel_in);
            break;
          case TASKDMX_DMX_PAN_MODE_VEL_POS:
            TaskMotor_vel_pos(TASKMOTOR_MOTOR_ADDR_0, x_vel_in);
            break;
          case TASKDMX_DMX_PAN_MODE_VEL_NEG:
            TaskMotor_vel_neg(TASKMOTOR_MOTOR_ADDR_0, x_vel_in);
            break;
          case TASKDMX_DMX_PAN_MODE_STOP_TILT:
            TaskMotor_stop_tilt();
            break;
          case TASKDMX_DMX_PAN_MODE_STOP:
            TaskMotor_stop();
            break;
          default:
            // error
            break;
        }
        if (pan_mode != TASKDMX_DMX_PAN_MODE_STOP_TILT && pan_mode != TASKDMX_DMX_PAN_MODE_STOP) {
          TaskMotor_goto_normal(TASKMOTOR_MOTOR_ADDR_1, y_in, 0);
        }
      }
    }
  }
}
