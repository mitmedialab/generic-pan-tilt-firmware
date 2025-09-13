/* task_motor.cpp
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
#include <SPI.h>
#include "task_motor.hpp"
#include "task_motor_reg_tmc5240.hpp"
#include "task_status_led.hpp"

extern "C" {
#include <TMC5240.h>
#include <TMC5240.c>
#include <TMC5240_HW_Abstraction.h>
}

/* Converts a time in ticks to a time in milliseconds.  This macro
 * overrides a macro of the same name defined in FreeRTOSConfig.h */
#ifndef pdTICKS_TO_MS
    #define pdTICKS_TO_MS( xTimeInTicks )    ( ( TickType_t ) ( ( ( uint64_t ) ( xTimeInTicks ) * ( uint64_t ) 1000U ) / ( uint64_t ) configTICK_RATE_HZ ) )
#endif

// Number of spots in queue
static const uint8_t TASKMOTOR_IO_QUEUE_SIZE = 10;

// Rate of motor task in milliseconds
static const uint16_t TASKMOTOR_WAKE_RATE_MS = 5;

// Rate of motor task in milliseconds
static const uint16_t TASKMOTOR_LAST_MESSAGE_TIMEOUT_LED_MS = 1000;

static const uint8_t MOT_PICO_PIN = 15;
static const uint8_t MOT_POCI_PIN = 12;
static const uint8_t MOT_SCK_PIN = 14;
static const uint8_t MOT0_CNTLR_CS_PIN = 13;
static const uint8_t MOT0_GPIO_CS_PIN = 22;
static const uint8_t MOT1_CNTLR_CS_PIN = 23;
static const uint8_t MOT1_GPIO_CS_PIN = 24;

static const uint32_t MICROSTEPS_PER_ROTATION_X = 200 * 4 * 256;
static const uint32_t MICROSTEPS_PER_ROTATION_Y = (100 * 256) - 1150;
static const int32_t MICROSTEPS_HOME_OFFSET_Y = -150;
static const uint32_t MOT0_VMAX = 500000;
static const uint32_t MOT1_VMAX = 100000;

static SPISettings spiSettingsTMC5240(1000000, MSBFIRST, SPI_MODE0);
static SPISettings spiSettingsMCP23S08(1000000, MSBFIRST, SPI_MODE0);

static const uint8_t MCP_SPI_ADDR = 0x40;
static const uint8_t MCP_DIR_REG = 0x00;
static const uint8_t MCP_VAL_REG = 0x09;

static QueueHandle_t TaskMotor_queue;

void tmc2540_reset(TaskMotor_motor_addr_t motorAddr);
uint32_t tmc2540_home_x(TaskMotor_motor_addr_t motorAddr);
uint32_t tmc2540_home_y(TaskMotor_motor_addr_t motorAddr);
void motor_vel_dir(TaskMotor_motor_addr_t motor_addr, TaskMotor_motor_direction_t dir, bool force_goto, uint32_t vel);
uint32_t motor_goto_dir(TaskMotor_motor_addr_t motor_addr, TaskMotor_motor_direction_t dir, uint16_t val_constrained, uint32_t prev_set_pos, bool force_goto, uint32_t range, uint32_t vel);

uint32_t map_better(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max) {
  return ((uint32_t)((uint64_t)x - (uint64_t)in_min) * ((uint64_t)out_max - (uint64_t)out_min) / ((uint64_t)in_max - (uint64_t)in_min) + (uint64_t)out_min);
}

void setupMCP(TaskMotor_mcp_addr_t mcp_addr) {
  uint8_t cs_pin = 0;
  switch (mcp_addr) {
    case TASKMOTOR_MOTOR_MCP_0:
      cs_pin = MOT0_GPIO_CS_PIN;
      break;
    case TASKMOTOR_MOTOR_MCP_1:
      cs_pin = MOT1_GPIO_CS_PIN;
      break;
    default:
      return;
  }
  // Setup pins
  uint8_t data1[] = {MCP_SPI_ADDR, MCP_DIR_REG, 0b11111100};
  size_t dataLength1 = 3;
  digitalWrite(cs_pin, LOW);
  SPI1.beginTransaction(spiSettingsMCP23S08);
  SPI1.transfer(data1, dataLength1);
  SPI1.endTransaction();
  digitalWrite(cs_pin, HIGH);

  // Unsleep and disable to allow safe spin down
  uint8_t data2[] = {MCP_SPI_ADDR, MCP_VAL_REG, 0b00000011};
  size_t dataLength2 = 3;
  digitalWrite(cs_pin, LOW);
  SPI1.beginTransaction(spiSettingsMCP23S08);
  SPI1.transfer(data2, dataLength2);
  SPI1.endTransaction();
  digitalWrite(cs_pin, HIGH);

  vTaskDelay(2000 / portTICK_PERIOD_MS);

  // Sleep to reset
  uint8_t data3[] = {MCP_SPI_ADDR, MCP_VAL_REG, 0b00000001};
  size_t dataLength3 = 3;
  digitalWrite(cs_pin, LOW);
  SPI1.beginTransaction(spiSettingsMCP23S08);
  SPI1.transfer(data3, dataLength3);
  SPI1.endTransaction();
  digitalWrite(cs_pin, HIGH);

  vTaskDelay(100 / portTICK_PERIOD_MS);

  // Unsleep
  uint8_t data4[] = {MCP_SPI_ADDR, MCP_VAL_REG, 0b00000011};
  size_t dataLength4 = 3;
  digitalWrite(cs_pin, LOW);
  SPI1.beginTransaction(spiSettingsMCP23S08);
  SPI1.transfer(data4, dataLength4);
  SPI1.endTransaction();
  digitalWrite(cs_pin, HIGH);

  vTaskDelay(100 / portTICK_PERIOD_MS);

  // Enable
  uint8_t data5[] = {MCP_SPI_ADDR, MCP_VAL_REG, 0b00000010};
  size_t dataLength5 = 3;
  digitalWrite(cs_pin, LOW);
  SPI1.beginTransaction(spiSettingsMCP23S08);
  SPI1.transfer(data5, dataLength5);
  SPI1.endTransaction();
  digitalWrite(cs_pin, HIGH);
}

void mcpDisableTMC(TaskMotor_mcp_addr_t mcp_addr) {
  uint8_t cs_pin = 0;
  switch (mcp_addr) {
    case TASKMOTOR_MOTOR_MCP_0:
      cs_pin = MOT0_GPIO_CS_PIN;
      break;
    case TASKMOTOR_MOTOR_MCP_1:
      cs_pin = MOT1_GPIO_CS_PIN;
      break;
    default:
      return;
  }
  // Setup pins
  uint8_t data1[] = {MCP_SPI_ADDR, MCP_VAL_REG, 0b00000011};
  size_t dataLength1 = 3;
  digitalWrite(cs_pin, LOW);
  SPI1.beginTransaction(spiSettingsMCP23S08);
  SPI1.transfer(data1, dataLength1);
  SPI1.endTransaction();
  digitalWrite(cs_pin, HIGH);
}

void mcpEnableTMC(TaskMotor_mcp_addr_t mcp_addr) {
  uint8_t cs_pin = 0;
  switch (mcp_addr) {
    case TASKMOTOR_MOTOR_MCP_0:
      cs_pin = MOT0_GPIO_CS_PIN;
      break;
    case TASKMOTOR_MOTOR_MCP_1:
      cs_pin = MOT1_GPIO_CS_PIN;
      break;
    default:
      return;
  }
  // Setup pins
  uint8_t data1[] = {MCP_SPI_ADDR, MCP_VAL_REG, 0b00000010};
  size_t dataLength1 = 3;
  digitalWrite(cs_pin, LOW);
  SPI1.beginTransaction(spiSettingsMCP23S08);
  SPI1.transfer(data1, dataLength1);
  SPI1.endTransaction();
  digitalWrite(cs_pin, HIGH);
}

// BEGIN TMC5240 WRAPPER
static inline int32_t tmc5240_readRegister(TaskMotor_motor_addr_t motorAddr, uint8_t address) {
  return tmc5240_readRegister((uint16_t)motorAddr, address);
}
static inline void tmc5240_writeRegister(TaskMotor_motor_addr_t motorAddr, uint8_t address, int32_t value) {
  tmc5240_writeRegister((uint16_t)motorAddr, address, value);
}
static inline uint32_t tmc5240_fieldRead(TaskMotor_motor_addr_t motorAddr, RegisterField field) {
  return tmc5240_fieldRead((uint16_t)motorAddr, field);
}
static inline void tmc5240_fieldWrite(TaskMotor_motor_addr_t motorAddr, RegisterField field, uint32_t value) {
  return tmc5240_fieldWrite((uint16_t)motorAddr, field, value);
}
TMC5240BusType tmc5240_getBusType(uint16_t icID) {
  return IC_BUS_SPI;
}
void tmc5240_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength) {
  uint8_t cs_pin = 0;
  switch ((TaskMotor_motor_addr_t)icID) {
    case TASKMOTOR_MOTOR_ADDR_0:
      cs_pin = MOT0_CNTLR_CS_PIN;
      break;
    case TASKMOTOR_MOTOR_ADDR_1:
      cs_pin = MOT1_CNTLR_CS_PIN;
      break;
    default:
      break;
  }

  digitalWrite(cs_pin, LOW);
  SPI1.beginTransaction(spiSettingsTMC5240);
  SPI1.transfer(data, dataLength);
  SPI1.endTransaction();
  digitalWrite(cs_pin, HIGH);
}
// UART
bool tmc5240_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength) {
  return false;
}
uint8_t tmc5240_getNodeAddress(uint16_t icID) {
  return 0;
}
// END TMC5240 WRAPPER

bool TaskMotor_setupQueue() {
  TaskMotor_queue = xQueueCreate(TASKMOTOR_IO_QUEUE_SIZE, sizeof(TaskMotor_queue_msg_t));
  if (TaskMotor_queue == 0) {
    return false;
  } else {
    return true;
  }
}

BaseType_t TaskMotor_goto_normal(TaskMotor_motor_addr_t motor, uint16_t pos, uint16_t vel) {
  TaskMotor_queue_msg_t msg = {
    .type = TASKMOTOR_MOTOR_MSG_TYPE_GOTO_NORMAL,
    .motor_addr = motor,
    .pos = pos,
    .vel = vel,
  };
  return xQueueSendToBack(TaskMotor_queue, (void *)&msg, (TickType_t)0);
}

BaseType_t TaskMotor_goto_shortest(TaskMotor_motor_addr_t motor, uint16_t pos, uint16_t vel) {
  TaskMotor_queue_msg_t msg = {
    .type = TASKMOTOR_MOTOR_MSG_TYPE_GOTO_SHORTEST,
    .motor_addr = motor,
    .pos = pos,
    .vel = vel,
  };
  return xQueueSendToBack(TaskMotor_queue, (void *)&msg, (TickType_t)0);
}

BaseType_t TaskMotor_goto_pos(TaskMotor_motor_addr_t motor, uint16_t pos, uint16_t vel) {
  TaskMotor_queue_msg_t msg = {
    .type = TASKMOTOR_MOTOR_MSG_TYPE_GOTO_POS,
    .motor_addr = motor,
    .pos = pos,
    .vel = vel,
  };
  return xQueueSendToBack(TaskMotor_queue, (void *)&msg, (TickType_t)0);
}

BaseType_t TaskMotor_goto_neg(TaskMotor_motor_addr_t motor, uint16_t pos, uint16_t vel) {
  TaskMotor_queue_msg_t msg = {
    .type = TASKMOTOR_MOTOR_MSG_TYPE_GOTO_NEG,
    .motor_addr = motor,
    .pos = pos,
    .vel = vel,
  };
  return xQueueSendToBack(TaskMotor_queue, (void *)&msg, (TickType_t)0);
}

BaseType_t TaskMotor_vel_pos(TaskMotor_motor_addr_t motor, uint16_t vel) {
  TaskMotor_queue_msg_t msg = {
    .type = TASKMOTOR_MOTOR_MSG_TYPE_VEL_POS,
    .motor_addr = motor,
    .pos = 0,
    .vel = vel,
  };
  return xQueueSendToBack(TaskMotor_queue, (void *)&msg, (TickType_t)0);
}

BaseType_t TaskMotor_vel_neg(TaskMotor_motor_addr_t motor, uint16_t vel) {
  TaskMotor_queue_msg_t msg = {
    .type = TASKMOTOR_MOTOR_MSG_TYPE_VEL_NEG,
    .motor_addr = motor,
    .pos = 0,
    .vel = vel,
  };
  return xQueueSendToBack(TaskMotor_queue, (void *)&msg, (TickType_t)0);
}

BaseType_t TaskMotor_stop_tilt() {
  TaskMotor_queue_msg_t msg = {
    .type = TASKMOTOR_MOTOR_MSG_TYPE_STOP_TILT,
    .motor_addr = TASKMOTOR_MOTOR_ADDR_1,
    .pos = 0,
    .vel = 0,
  };
  return xQueueSendToBack(TaskMotor_queue, (void *)&msg, (TickType_t)0);
}

BaseType_t TaskMotor_stop() {
  TaskMotor_queue_msg_t msg = {
    .type = TASKMOTOR_MOTOR_MSG_TYPE_STOP,
    .motor_addr = TASKMOTOR_MOTOR_ADDR_0,
    .pos = 0,
    .vel = 0,
  };
  return xQueueSendToBack(TaskMotor_queue, (void *)&msg, (TickType_t)0);
}

// https://www.analog.com/en/resources/app-notes/an-028.html
const int32_t DISPLACEMENT_CONST_X = UINT32_MAX % MICROSTEPS_PER_ROTATION_X;
static int32_t offset_x = 0;
void correctPositionX() {
  const uint32_t current_pos = tmc5240_readRegister(TASKMOTOR_MOTOR_ADDR_0, TMC5240_XACTUAL);
  static uint32_t prev_pos = current_pos;
  if (prev_pos & 0xC0000000 == 0x00000000 && current_pos & 0xC0000000 == 0xC0000000) {
    // underflow
    offset_x -= DISPLACEMENT_CONST_X;
  } else if (prev_pos & 0xC0000000 == 0xC0000000 && current_pos & 0xC0000000 == 0x00000000) {
    // overflow
    offset_x += DISPLACEMENT_CONST_X;
  }
  offset_x = offset_x % MICROSTEPS_PER_ROTATION_X;
  prev_pos = current_pos;
}

uint32_t getCorrectedPositionX(uint32_t xactual) {
  return (xactual + offset_x) % MICROSTEPS_PER_ROTATION_X;
}

void TaskMotor(void *pvParameters) {
  (void)pvParameters;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  pinMode(MOT_PICO_PIN, OUTPUT);
  pinMode(MOT_POCI_PIN, INPUT);
  pinMode(MOT_SCK_PIN, OUTPUT);
  pinMode(MOT0_CNTLR_CS_PIN, OUTPUT);
  pinMode(MOT0_GPIO_CS_PIN, OUTPUT);
  pinMode(MOT1_CNTLR_CS_PIN, OUTPUT);
  pinMode(MOT1_GPIO_CS_PIN, OUTPUT);
  digitalWrite(MOT0_CNTLR_CS_PIN, HIGH);
  digitalWrite(MOT0_GPIO_CS_PIN, HIGH);
  digitalWrite(MOT1_CNTLR_CS_PIN, HIGH);
  digitalWrite(MOT1_GPIO_CS_PIN, HIGH);
  SPI1.setTX(MOT_PICO_PIN);
  SPI1.setRX(MOT_POCI_PIN);
  SPI1.setSCK(MOT_SCK_PIN);
  SPI1.begin();

  setupMCP(TASKMOTOR_MOTOR_MCP_0);
  setupMCP(TASKMOTOR_MOTOR_MCP_1);

  tmc2540_reset(TASKMOTOR_MOTOR_ADDR_0);
  tmc2540_reset(TASKMOTOR_MOTOR_ADDR_1);
  vTaskDelayUntil(&xLastWakeTime, (2000 / portTICK_PERIOD_MS));
  uint32_t x_range = tmc2540_home_x(TASKMOTOR_MOTOR_ADDR_0);
  Serial1.print("x range: ");
  Serial1.println(x_range);
  uint32_t y_range = tmc2540_home_y(TASKMOTOR_MOTOR_ADDR_1);
  Serial1.print("y range: ");
  Serial1.println(y_range);
  TaskStatusLED_setState(TASKSTATUSLED_LED_STATE_GREEN_BLINK_SLOW);

  tmc5240_writeRegister(TASKMOTOR_MOTOR_ADDR_0, TMC5240_AMAX, 20000);
  tmc5240_writeRegister(TASKMOTOR_MOTOR_ADDR_0, TMC5240_DMAX, 5000);
  tmc5240_writeRegister(TASKMOTOR_MOTOR_ADDR_1, TMC5240_AMAX, 2000);
  tmc5240_writeRegister(TASKMOTOR_MOTOR_ADDR_1, TMC5240_DMAX, 2000);

  tmc5240_writeRegister(TASKMOTOR_MOTOR_ADDR_0, TMC5240_XTARGET, 0);
  tmc5240_fieldWrite(TASKMOTOR_MOTOR_ADDR_0, TMC5240_RAMPMODE_FIELD, TMC5240_MODE_POSITION);
  tmc5240_writeRegister(TASKMOTOR_MOTOR_ADDR_0, TMC5240_VMAX, MOT0_VMAX);

  tmc5240_writeRegister(TASKMOTOR_MOTOR_ADDR_1, TMC5240_XTARGET, 0);
  tmc5240_fieldWrite(TASKMOTOR_MOTOR_ADDR_1, TMC5240_RAMPMODE_FIELD, TMC5240_MODE_POSITION);
  tmc5240_writeRegister(TASKMOTOR_MOTOR_ADDR_1, TMC5240_VMAX, MOT1_VMAX);

  static TaskMotor_queue_msg_t currentMsg;

  // Discard old messages
  TaskMotor_queue_msg_t msg = {
    .type = TASKMOTOR_MOTOR_MSG_TYPE_CLEAR_COMPLETE,
    .motor_addr = TASKMOTOR_MOTOR_ADDR_0,
    .pos = 0,
    .vel = 0,
  };
  while (xQueueSendToBack(TaskMotor_queue, (void *)&msg, (TickType_t)0) == errQUEUE_FULL) { 
    xQueueReceive(TaskMotor_queue, (void *)&currentMsg, (TickType_t)0);
  }

  bool clearComplete = false;
  static uint32_t prev_set_pos_0 = 0;
  uint32_t last_msg_time = pdTICKS_TO_MS(xTaskGetTickCount());
  while (1) {
    correctPositionX();
    static TaskMotor_queue_msg_t prev_msg_x = msg;
    static TaskMotor_queue_msg_t prev_msg_y = msg;
    while (xQueueReceive(TaskMotor_queue, (void *)&currentMsg, (TickType_t)0) == pdTRUE) {
      last_msg_time = pdTICKS_TO_MS(xTaskGetTickCount());

      bool force_goto_x = (prev_msg_x.type != currentMsg.type);
      uint32_t vel_mapped = map_better((uint32_t)currentMsg.vel, (uint32_t)0, (uint32_t)UINT16_MAX, (uint32_t)0, (uint32_t)MOT0_VMAX);
      switch (currentMsg.type) {
        case TASKMOTOR_MOTOR_MSG_TYPE_CLEAR_COMPLETE:
          clearComplete = true;
          break;
        case TASKMOTOR_MOTOR_MSG_TYPE_GOTO_NORMAL:
          if (clearComplete) {
            switch (currentMsg.motor_addr) {
              case TASKMOTOR_MOTOR_ADDR_0:
                prev_set_pos_0 = motor_goto_dir(currentMsg.motor_addr, TASKMOTOR_MOTOR_DIRECTION_ANY, currentMsg.pos, prev_set_pos_0, force_goto_x, x_range, vel_mapped);
                break;
              case TASKMOTOR_MOTOR_ADDR_1:
                mcpEnableTMC(TASKMOTOR_MOTOR_MCP_1);
                tmc5240_writeRegister(currentMsg.motor_addr, TMC5240_XTARGET, map_better((uint32_t)currentMsg.pos, (uint32_t)0, (uint32_t)UINT16_MAX, (uint32_t)0, (uint32_t)y_range));
                tmc5240_fieldWrite(currentMsg.motor_addr, TMC5240_RAMPMODE_FIELD, TMC5240_MODE_POSITION);
                tmc5240_writeRegister(currentMsg.motor_addr, TMC5240_VMAX, MOT1_VMAX);
                break;
              default:
                // unknown motor
                break;
            }
          }
          break;
        case TASKMOTOR_MOTOR_MSG_TYPE_GOTO_SHORTEST:
          if (clearComplete) {
            switch (currentMsg.motor_addr) {
              case TASKMOTOR_MOTOR_ADDR_0:
                prev_set_pos_0 = motor_goto_dir(currentMsg.motor_addr, TASKMOTOR_MOTOR_DIRECTION_SHORTEST, currentMsg.pos, prev_set_pos_0, force_goto_x, x_range, vel_mapped);
                break;
              default:
                // invalid motor
                break;
            }
          }
          break;
        case TASKMOTOR_MOTOR_MSG_TYPE_GOTO_POS:
          if (clearComplete) {
            switch (currentMsg.motor_addr) {
              case TASKMOTOR_MOTOR_ADDR_0:
                prev_set_pos_0 = motor_goto_dir(currentMsg.motor_addr, TASKMOTOR_MOTOR_DIRECTION_POS, currentMsg.pos, prev_set_pos_0, force_goto_x, x_range, vel_mapped);
                break;
              default:
                // invalid motor
                break;
            }
          }
          break;
        case TASKMOTOR_MOTOR_MSG_TYPE_GOTO_NEG:
          if (clearComplete) {
            switch (currentMsg.motor_addr) {
              case TASKMOTOR_MOTOR_ADDR_0:
                prev_set_pos_0 = motor_goto_dir(currentMsg.motor_addr, TASKMOTOR_MOTOR_DIRECTION_NEG, currentMsg.pos, prev_set_pos_0, force_goto_x, x_range, vel_mapped);
                break;
              default:
                // invalid motor
                break;
            }
          }
          break;
        case TASKMOTOR_MOTOR_MSG_TYPE_VEL_POS:
          if (clearComplete) {
            switch (currentMsg.motor_addr) {
              case TASKMOTOR_MOTOR_ADDR_0:
                motor_vel_dir(currentMsg.motor_addr, TASKMOTOR_MOTOR_DIRECTION_POS, false, vel_mapped);
                break;
              default:
                // invalid motor
                break;
            }
          }
          break;
        case TASKMOTOR_MOTOR_MSG_TYPE_VEL_NEG:
          if (clearComplete) {
            switch (currentMsg.motor_addr) {
              case TASKMOTOR_MOTOR_ADDR_0:
                motor_vel_dir(currentMsg.motor_addr, TASKMOTOR_MOTOR_DIRECTION_NEG, false, vel_mapped);
                break;
              default:
                // invalid motor
                break;
            }
          }
          break;
        case TASKMOTOR_MOTOR_MSG_TYPE_STOP_TILT:
          if (clearComplete) {
            switch (currentMsg.motor_addr) {
              case TASKMOTOR_MOTOR_ADDR_1:
                tmc5240_writeRegister(TASKMOTOR_MOTOR_ADDR_1, TMC5240_VMAX, 0);
                mcpDisableTMC(TASKMOTOR_MOTOR_MCP_1);
                break;
              default:
                // invalid motor
                break;
            }
          }
          break;
        case TASKMOTOR_MOTOR_MSG_TYPE_STOP:
          if (clearComplete) {
            switch (currentMsg.motor_addr) {
              case TASKMOTOR_MOTOR_ADDR_0:
                tmc5240_writeRegister(TASKMOTOR_MOTOR_ADDR_0, TMC5240_VMAX, 0);
                tmc5240_writeRegister(TASKMOTOR_MOTOR_ADDR_1, TMC5240_VMAX, 0);
                mcpDisableTMC(TASKMOTOR_MOTOR_MCP_0);
                mcpDisableTMC(TASKMOTOR_MOTOR_MCP_1);
                break;
              default:
                // invalid motor
                break;
            }
          }
          break;
        default:
          // unknown type
          break;
      }
      if (currentMsg.motor_addr == TASKMOTOR_MOTOR_ADDR_0) {
        prev_msg_x = currentMsg;
      } else if (currentMsg.motor_addr == TASKMOTOR_MOTOR_ADDR_1) {
        prev_msg_y = currentMsg;
      }
    }

    if (pdTICKS_TO_MS(xTaskGetTickCount()) - last_msg_time >= TASKMOTOR_LAST_MESSAGE_TIMEOUT_LED_MS) {
      TaskStatusLED_setState(TASKSTATUSLED_LED_STATE_GREEN_BLINK_SLOW);
    } else {
      TaskStatusLED_setState(TASKSTATUSLED_LED_STATE_OFF);
    }

    vTaskDelayUntil(&xLastWakeTime, (TASKMOTOR_WAKE_RATE_MS / portTICK_PERIOD_MS));
  }
}

void motor_vel_dir(TaskMotor_motor_addr_t motor_addr, TaskMotor_motor_direction_t dir, bool force_goto, uint32_t vel) {
  taskENTER_CRITICAL();
  switch (motor_addr) {
    case TASKMOTOR_MOTOR_ADDR_0:
      mcpEnableTMC(TASKMOTOR_MOTOR_MCP_0);
      break;
    case TASKMOTOR_MOTOR_ADDR_1:
      mcpEnableTMC(TASKMOTOR_MOTOR_MCP_1);
      break;
    default:
      // unknown motor
      break;
  }
  switch(dir) {
    case TASKMOTOR_MOTOR_DIRECTION_POS:
      tmc5240_fieldWrite(motor_addr, TMC5240_RAMPMODE_FIELD, TMC5240_MODE_VELPOS);
      break;
    case TASKMOTOR_MOTOR_DIRECTION_NEG:
      tmc5240_fieldWrite(motor_addr, TMC5240_RAMPMODE_FIELD, TMC5240_MODE_VELNEG);
      break;
    default:
      // invalid option
      break;
  }
  tmc5240_writeRegister(motor_addr, TMC5240_VMAX, vel);
  taskEXIT_CRITICAL();
}

uint32_t motor_goto_dir(TaskMotor_motor_addr_t motor_addr, TaskMotor_motor_direction_t dir, uint16_t val_constrained, uint32_t prev_set_pos, bool force_goto, uint32_t range, uint32_t vel) {
  uint32_t mapped_pos = map_better((uint32_t)val_constrained, (uint32_t)0, (uint32_t)UINT16_MAX, (uint32_t)0, (uint32_t)range);
  taskENTER_CRITICAL();
  if (force_goto || mapped_pos != prev_set_pos) {
    const uint32_t current_pos = tmc5240_readRegister(motor_addr, TMC5240_XACTUAL);
    const uint32_t current_pos_indexed = getCorrectedPositionX(current_pos);
    uint32_t goto_pos = current_pos;
    switch(dir) {
      case TASKMOTOR_MOTOR_DIRECTION_ANY:
        if (mapped_pos >= current_pos_indexed) {
          goto_pos += mapped_pos - current_pos_indexed;
        } else {
          goto_pos -= current_pos_indexed - mapped_pos;
        }
        break;
      case TASKMOTOR_MOTOR_DIRECTION_SHORTEST:
        if (mapped_pos >= current_pos_indexed) {
          if (mapped_pos - current_pos_indexed <= range - (mapped_pos - current_pos_indexed)) {
            goto_pos += mapped_pos - current_pos_indexed;
          } else {
            goto_pos -= range - (mapped_pos - current_pos_indexed);
          }
        } else {
          if (current_pos_indexed - mapped_pos <= range - (current_pos_indexed - mapped_pos)) {
            goto_pos -= current_pos_indexed - mapped_pos;
          } else {
            goto_pos += range - (current_pos_indexed - mapped_pos);
          }
        }
        break;
      case TASKMOTOR_MOTOR_DIRECTION_POS:
        if (mapped_pos >= current_pos_indexed) {
          goto_pos += mapped_pos - current_pos_indexed;
        } else {
          goto_pos += range - (current_pos_indexed - mapped_pos);
        }
        break;
      case TASKMOTOR_MOTOR_DIRECTION_NEG:
        if (mapped_pos <= current_pos_indexed) {
          goto_pos -= current_pos_indexed - mapped_pos;
        } else {
          goto_pos -= range - (mapped_pos - current_pos_indexed);
        }
        break;
      default:
        // unknown
        return mapped_pos;
    }
    switch (motor_addr) {
      case TASKMOTOR_MOTOR_ADDR_0:
        mcpEnableTMC(TASKMOTOR_MOTOR_MCP_0);
        break;
      case TASKMOTOR_MOTOR_ADDR_1:
        mcpEnableTMC(TASKMOTOR_MOTOR_MCP_1);
        break;
      default:
        // unknown motor
        break;
    }
    tmc5240_writeRegister(motor_addr, TMC5240_XTARGET, goto_pos);
    tmc5240_fieldWrite(motor_addr, TMC5240_RAMPMODE_FIELD, TMC5240_MODE_POSITION);
  }
  tmc5240_writeRegister(motor_addr, TMC5240_VMAX, vel);
  taskEXIT_CRITICAL();
  
  return mapped_pos;
}

void tmc2540_reset(TaskMotor_motor_addr_t motorAddr) {
  uint32_t gconf = 0;
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_FAST_STANDSTILL_FIELD, 0);         // 1 Normal time
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_EN_PWM_MODE_FIELD, 1);             // 2 enable StealthChop2
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_SHAFT_FIELD, 0);                   // 4 normal direction
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_DIAG0_STALL_STEP_FIELD, 0);        // 7 DIAG0 out
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_DIAG1_STALL_DIR_FIELD, 0);         // 8 DIAG1 out
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_DIAG0_INT_PUSHPULL_FIELD, 0);      // 12 DIAG0 out active low
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_DIAG1_POSCOMP_PUSHPULL_FIELD, 0);  // 13 DIAG1 out active low
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_SMALL_HYSTERESIS_FIELD, 1);        // 14 hysteresis 1/32
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_STOP_ENABLE_FIELD, 0);             // 15 do not hard stop on enca trigger
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_DIRECT_MODE_FIELD, 0);             // 16 disable direct motor phase control through serial
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_LENGTH_STEP_PULSE_FIELD, 0);       // 17 length step pulse
  tmc5240_writeRegister(motorAddr, TMC5240_GCONF, gconf);

  uint32_t gstat = 0;
  gstat = tmc5240_fieldUpdate(gstat, TMC5240_RESET_FIELD, 1);           // 0 clear reset flag
  gstat = tmc5240_fieldUpdate(gstat, TMC5240_DRV_ERR_FIELD, 1);         // 1 clear error flag
  gstat = tmc5240_fieldUpdate(gstat, TMC5240_UV_CP_FIELD, 1);           // 2 clear charge pump flag
  gstat = tmc5240_fieldUpdate(gstat, TMC5240_REGISTER_RESET_FIELD, 1);  // 3 clear reg map reset flag
  gstat = tmc5240_fieldUpdate(gstat, TMC5240_VM_UVLO_FIELD, 1);         // 4 clear error flag
  tmc5240_writeRegister(motorAddr, TMC5240_GSTAT, gstat);

  uint32_t drvconf = 0;
  drvconf = tmc5240_fieldUpdate(drvconf, TMC5240_CURRENT_RANGE_FIELD, 0);  // 0 current range 1A
  drvconf = tmc5240_fieldUpdate(drvconf, TMC5240_SLOPE_CONTROL_FIELD, 0);  // 1 slope control 100V/us
  tmc5240_writeRegister(motorAddr, TMC5240_DRV_CONF, drvconf);

  tmc5240_fieldWrite(motorAddr, TMC5240_GLOBAL_SCALER_FIELD, 0);  // full-scale

  uint32_t iholdirun = 0;
  iholdirun = tmc5240_fieldUpdate(iholdirun, TMC5240_IHOLD_FIELD, 8);       // 0 ihold
  iholdirun = tmc5240_fieldUpdate(iholdirun, TMC5240_IRUN_FIELD, 31);       // 1 irun
  iholdirun = tmc5240_fieldUpdate(iholdirun, TMC5240_IHOLDDELAY_FIELD, 0x1);  // 2 iholddelay power down delay
  iholdirun = tmc5240_fieldUpdate(iholdirun, TMC5240_IRUNDELAY_FIELD, 0x4);   // 3 irundelay duration of motor power up
  tmc5240_writeRegister(motorAddr, TMC5240_IHOLD_IRUN, iholdirun);

  tmc5240_fieldWrite(motorAddr, TMC5240_TPOWERDOWN_FIELD, 10);  // motor current power down delay after standstill
  tmc5240_fieldWrite(motorAddr, TMC5240_TPWMTHRS_FIELD, 0);     // stealthchop upper velocity
  tmc5240_fieldWrite(motorAddr, TMC5240_TCOOLTHRS_FIELD, 0);    //coolstep and stallguard lower velocity
  tmc5240_fieldWrite(motorAddr, TMC5240_THIGH_FIELD, 0);        // chopper velocity dependent switching

  tmc5240_fieldWrite(motorAddr, TMC5240_RAMPMODE_FIELD, TMC5240_MODE_HOLD);  // ramp mode hold
  tmc5240_writeRegister(motorAddr, TMC5240_XACTUAL, 0);                      // zero out position
  tmc5240_fieldWrite(motorAddr, TMC5240_VSTART_FIELD, 0);                    // start vel
  tmc5240_fieldWrite(motorAddr, TMC5240_A1_FIELD, 0);                        // accel 1
  tmc5240_fieldWrite(motorAddr, TMC5240_V1_FIELD, 0);                        // vel 1
  tmc5240_fieldWrite(motorAddr, TMC5240_AMAX_FIELD, 0);                      // accel max
  tmc5240_fieldWrite(motorAddr, TMC5240_VMAX_FIELD, 0);                      // vel max
  tmc5240_fieldWrite(motorAddr, TMC5240_DMAX_FIELD, 0);                      // deaccel max
  tmc5240_fieldWrite(motorAddr, TMC5240_TVMAX_FIELD, 0);                     // jerk reduce constant vel switcihng speed up/down
  tmc5240_fieldWrite(motorAddr, TMC5240_D1_FIELD, 0xA);                      // deaccel 1 DO NOT SET 0 IN POS MODE
  tmc5240_fieldWrite(motorAddr, TMC5240_VSTOP_FIELD, 0xA);                   // stop vel DO NOT SET 0 IN POS MODE
  tmc5240_fieldWrite(motorAddr, TMC5240_TZEROWAIT_FIELD, 0);                 // wait time to next movement
  tmc5240_fieldWrite(motorAddr, TMC5240_XTARGET_FIELD, 0);                   // wait time to next movement
  tmc5240_writeRegister(motorAddr, TMC5240_XTARGET, 0);                      // target pos
  tmc5240_fieldWrite(motorAddr, TMC5240_V2_FIELD, 0);                        // vel 2
  tmc5240_fieldWrite(motorAddr, TMC5240_A2_FIELD, 0);                        // accel 2
  tmc5240_fieldWrite(motorAddr, TMC5240_D2_FIELD, 0xA);                      // deaccel 2 DO NOT SET 0 IN POS MODE
  tmc5240_fieldWrite(motorAddr, TMC5240_VDCMIN_FIELD, 0);                    // dcstep mode

  uint32_t swmode = 0;
  swmode = tmc5240_fieldUpdate(swmode, TMC5240_STOP_L_ENABLE_FIELD, 0);      // 00 stop when left active
  swmode = tmc5240_fieldUpdate(swmode, TMC5240_STOP_R_ENABLE_FIELD, 0);      // 01 stop when right active
  swmode = tmc5240_fieldUpdate(swmode, TMC5240_POL_STOP_L_FIELD, 1);         // 02 left switch invert
  swmode = tmc5240_fieldUpdate(swmode, TMC5240_POL_STOP_R_FIELD, 1);         // 03 right switch invert
  swmode = tmc5240_fieldUpdate(swmode, TMC5240_SWAP_LR_FIELD, 0);            // 04 swap left and right
  swmode = tmc5240_fieldUpdate(swmode, TMC5240_LATCH_L_ACTIVE_FIELD, 1);     // 05 l latch rising edge
  swmode = tmc5240_fieldUpdate(swmode, TMC5240_LATCH_L_INACTIVE_FIELD, 0);   // 06 l latch falling edge
  swmode = tmc5240_fieldUpdate(swmode, TMC5240_LATCH_R_ACTIVE_FIELD, 0);     // 07 r latch rising edge
  swmode = tmc5240_fieldUpdate(swmode, TMC5240_LATCH_R_INACTIVE_FIELD, 0);   // 08 r latch falling edge
  swmode = tmc5240_fieldUpdate(swmode, TMC5240_EN_LATCH_ENCODER_FIELD, 0);   // 09 latch encoder on switch event
  swmode = tmc5240_fieldUpdate(swmode, TMC5240_SG_STOP_FIELD, 0);            // 10 stop by stallguard2
  swmode = tmc5240_fieldUpdate(swmode, TMC5240_EN_SOFTSTOP_FIELD, 0);        // 11 0 hardstop 1 softstop
  swmode = tmc5240_fieldUpdate(swmode, TMC5240_EN_VIRTUAL_STOP_L_FIELD, 0);  // 12 stop motor left virtual
  swmode = tmc5240_fieldUpdate(swmode, TMC5240_EN_VIRTUAL_STOP_R_FIELD, 0);  // 13 stop motor right virtual
  swmode = tmc5240_fieldUpdate(swmode, TMC5240_VIRTUAL_STOP_ENC_FIELD, 0);   // 14 0 virtual stop xactual 1 virtual stop encoder
  tmc5240_writeRegister(motorAddr, TMC5240_SWMODE, swmode);

  tmc5240_writeRegister(motorAddr, TMC5240_RAMPSTAT, 0xff);  // clear RAMP_STAT

  uint32_t chopconf = 0;
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_TOFF_FIELD, 0);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_TFD_ALL_FIELD, 0x5);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_HEND_OFFSET_FIELD, 0x2);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_FD3_FIELD, 0);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_DISFDCC_FIELD, 0);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_CHM_FIELD, 0); // ?
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_TBL_FIELD, 0b10);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_VHIGHFS_FIELD, 0); // ?
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_VHIGHCHM_FIELD, 0); // ?
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_TPFD_FIELD, 0x4);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_MRES_FIELD, 0);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_INTPOL_FIELD, 0x1);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_DEDGE_FIELD, 0); // reserved
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_DISS2G_FIELD, 0x0);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_DISS2VS_FIELD, 0x1);
  tmc5240_writeRegister(motorAddr, TMC5240_CHOPCONF, chopconf);

  uint32_t pwmconf = 0;
  pwmconf = tmc5240_fieldUpdate(pwmconf, TMC5240_PWM_OFS_FIELD, 0x1D);
  pwmconf = tmc5240_fieldUpdate(pwmconf, TMC5240_PWM_GRAD_FIELD, 0x0);
  pwmconf = tmc5240_fieldUpdate(pwmconf, TMC5240_PWM_FREQ_FIELD, 0x0);
  pwmconf = tmc5240_fieldUpdate(pwmconf, TMC5240_PWM_AUTOSCALE_FIELD, 0x1);
  pwmconf = tmc5240_fieldUpdate(pwmconf, TMC5240_PWM_AUTOGRAD_FIELD, 0x1);
  pwmconf = tmc5240_fieldUpdate(pwmconf, TMC5240_FREEWHEEL_FIELD, 0x0);
  pwmconf = tmc5240_fieldUpdate(pwmconf, TMC5240_PWM_MEAS_SD_ENABLE_FIELD, 0x0);
  pwmconf = tmc5240_fieldUpdate(pwmconf, TMC5240_PWM_DIS_REG_STST_FIELD, 0x0);
  pwmconf = tmc5240_fieldUpdate(pwmconf, TMC5240_PWM_REG_FIELD, 0x4);
  pwmconf = tmc5240_fieldUpdate(pwmconf, TMC5240_PWM_LIM_FIELD, 0xC);
  tmc5240_writeRegister(motorAddr, TMC5240_PWMCONF, pwmconf);

  // default vstop/vstart
  tmc5240_writeRegister(motorAddr, TMC5240_VSTOP, 10);
  tmc5240_writeRegister(motorAddr, TMC5240_VSTART, 0);

  // disable V1, V2 portions of trapezoid
  tmc5240_writeRegister(motorAddr, TMC5240_V1, 0);
  tmc5240_writeRegister(motorAddr, TMC5240_V2, 0);
  tmc5240_writeRegister(motorAddr, TMC5240_A1, 0);
  tmc5240_writeRegister(motorAddr, TMC5240_A2, 0);
  tmc5240_writeRegister(motorAddr, TMC5240_D1, DEFAULT_D1_D2);
  tmc5240_writeRegister(motorAddr, TMC5240_D2, DEFAULT_D1_D2);
  tmc5240_writeRegister(motorAddr, TMC5240_VDCMIN, 0);

  tmc5240_writeRegister(motorAddr, TMC5240_THIGH, 0);
  tmc5240_writeRegister(motorAddr, TMC5240_TVMAX, 0);
  tmc5240_writeRegister(motorAddr, TMC5240_TCOOLTHRS, 0);
  tmc5240_writeRegister(motorAddr, TMC5240_TPWMTHRS, 0);

  // set ramp-up and ramp-down accel
  tmc5240_writeRegister(motorAddr, TMC5240_AMAX, DEFAULT_ACCEL);
  tmc5240_writeRegister(motorAddr, TMC5240_DMAX, DEFAULT_ACCEL);

  // set to hold at 0 velocity
  tmc5240_writeRegister(motorAddr, TMC5240_VMAX, 0);
  tmc5240_fieldWrite(motorAddr, TMC5240_RAMPMODE_FIELD, TMC5240_MODE_HOLD);
}

uint32_t tmc2540_home_x(TaskMotor_motor_addr_t motorAddr) {
  TaskStatusLED_setState(TASKSTATUSLED_LED_STATE_ORANGE_BLINK_SLOW);
  uint32_t gconf = 0;
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_FAST_STANDSTILL_FIELD, 0);         // 1 Normal time
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_EN_PWM_MODE_FIELD, 1);             // 2 enable StealthChop2
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_SHAFT_FIELD, 0);                   // 4 normal direction
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_DIAG0_STALL_STEP_FIELD, 0);        // 7 DIAG0 out
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_DIAG1_STALL_DIR_FIELD, 0);         // 8 DIAG1 out
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_DIAG0_INT_PUSHPULL_FIELD, 0);      // 12 DIAG0 out active low
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_DIAG1_POSCOMP_PUSHPULL_FIELD, 0);  // 13 DIAG1 out active low
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_SMALL_HYSTERESIS_FIELD, 1);        // 14 hysteresis 1/32
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_STOP_ENABLE_FIELD, 0);             // 15 do not hard stop on enca trigger
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_DIRECT_MODE_FIELD, 0);             // 16 disable direct motor phase control through serial
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_LENGTH_STEP_PULSE_FIELD, 0);       // 17 length step pulse
  tmc5240_writeRegister(motorAddr, TMC5240_GCONF, gconf);

  uint32_t drvconf = 0;
  drvconf = tmc5240_fieldUpdate(drvconf, TMC5240_CURRENT_RANGE_FIELD, 0);       // 0 current range 1A
  drvconf = tmc5240_fieldUpdate(drvconf, TMC5240_CURRENT_RANGE_FIELD, 0b01);    // 0 current range 2A
  drvconf = tmc5240_fieldUpdate(drvconf, TMC5240_SLOPE_CONTROL_FIELD, 0);       // 1 slope control 100V/us
  tmc5240_writeRegister(motorAddr, TMC5240_DRV_CONF, drvconf);

  tmc5240_fieldWrite(motorAddr, TMC5240_GLOBAL_SCALER_FIELD, 190);  // 1.5A

  uint32_t iholdirun = 0;
  iholdirun = tmc5240_fieldUpdate(iholdirun, TMC5240_IHOLD_FIELD, 8);           // 0 ihold
  iholdirun = tmc5240_fieldUpdate(iholdirun, TMC5240_IHOLD_FIELD, 21);          // 0 ihold 70% of irun
  iholdirun = tmc5240_fieldUpdate(iholdirun, TMC5240_IRUN_FIELD, 31);           // 1 irun
  iholdirun = tmc5240_fieldUpdate(iholdirun, TMC5240_IHOLDDELAY_FIELD, 0x1);    // 2 iholddelay power down delay
  iholdirun = tmc5240_fieldUpdate(iholdirun, TMC5240_IRUNDELAY_FIELD, 0x4);     // 3 irundelay duration of motor power up
  tmc5240_writeRegister(motorAddr, TMC5240_IHOLD_IRUN, iholdirun);

  uint32_t chopconf = 0;
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_TOFF_FIELD, 0b0011);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_TFD_ALL_FIELD, 0b101); // HSTRT
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_HEND_OFFSET_FIELD, 0b0010);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_FD3_FIELD, 0);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_DISFDCC_FIELD, 0);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_CHM_FIELD, 0); // ?
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_TBL_FIELD, 0x2);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_VHIGHFS_FIELD, 0); // ?
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_VHIGHCHM_FIELD, 0); // ?
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_TPFD_FIELD, 0x4);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_MRES_FIELD, 0);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_INTPOL_FIELD, 0x1);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_DEDGE_FIELD, 0); // reserved
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_DISS2G_FIELD, 0x0);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_DISS2VS_FIELD, 0x0);
  tmc5240_writeRegister(motorAddr, TMC5240_CHOPCONF, chopconf);

  // stealthchop
  tmc5240_fieldWrite(motorAddr, TMC5240_EN_PWM_MODE_FIELD, 1);
  tmc5240_fieldWrite(motorAddr, TMC5240_PWM_AUTOSCALE_FIELD, 1);
  tmc5240_fieldWrite(motorAddr, TMC5240_PWM_AUTOGRAD_FIELD, 1);
  tmc5240_fieldWrite(motorAddr, TMC5240_PWM_MEAS_SD_ENABLE_FIELD, 1);
  tmc5240_fieldWrite(motorAddr, TMC5240_PWM_FREQ_FIELD, 0);
  tmc5240_fieldWrite(motorAddr, TMC5240_TOFF_FIELD, 3);
  tmc5240_fieldWrite(motorAddr, TMC5240_TBL_FIELD, 2);
  tmc5240_fieldWrite(motorAddr, TMC5240_TFD_ALL_FIELD, 4); // HSTRT
  tmc5240_fieldWrite(motorAddr, TMC5240_HEND_OFFSET_FIELD, 0);
  

  tmc5240_writeRegister(motorAddr, TMC5240_AMAX, 51200);  // writing value to address 5 = 0x20(AMAX)

  // Enable motor driver
  tmc5240_writeRegister(motorAddr, TMC5240_GCONF, 0x10020002);  // writing value 0x10020002 = 268566530 = 0.0 to address 0 = 0x00(GCONF)

  tmc5240_writeRegister(motorAddr, TMC5240_AMAX, 10000);
  tmc5240_writeRegister(motorAddr, TMC5240_DMAX, 10000);

  // Quickly find home (inaccurate)
  tmc5240_fieldWrite(motorAddr, TMC5240_STOP_L_ENABLE_FIELD, 1); // stop when hit left
  tmc5240_fieldWrite(motorAddr, TMC5240_RAMPMODE_FIELD, TMC5240_MODE_VELNEG); // will only stop for left in neg dir
  tmc5240_writeRegister(motorAddr, TMC5240_VMAX, 75000);
  while (tmc5240_fieldRead(motorAddr, TMC5240_EVENT_STOP_L_FIELD) == 0) {
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  tmc5240_writeRegister(motorAddr, TMC5240_VMAX, 0);
  tmc5240_fieldWrite(motorAddr, TMC5240_STOP_L_ENABLE_FIELD, 0); // disable stop when hit left

  // move right to get out of the way
  tmc5240_fieldWrite(motorAddr, TMC5240_RAMPMODE_FIELD, TMC5240_MODE_VELPOS);
  tmc5240_writeRegister(motorAddr, TMC5240_VMAX, 10000);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  tmc5240_writeRegister(motorAddr, TMC5240_VMAX, 0);

  // HOME
  // Stop on left
  tmc5240_fieldWrite(motorAddr, TMC5240_STOP_L_ENABLE_FIELD, 1); // stop when hit left
  tmc5240_fieldWrite(motorAddr, TMC5240_LATCH_L_ACTIVE_FIELD, 1); // save val when hit left
  tmc5240_fieldWrite(motorAddr, TMC5240_RAMPMODE_FIELD, TMC5240_MODE_VELNEG); // will only stop for left in neg dir
  tmc5240_writeRegister(motorAddr, TMC5240_VMAX, 10000);
  while (tmc5240_fieldRead(motorAddr, TMC5240_EVENT_STOP_L_FIELD) == 0) {
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  // found left, save it
  vTaskDelay(250 / portTICK_PERIOD_MS); // stop for a moment
  uint32_t left_val = tmc5240_readRegister(motorAddr, TMC5240_XLATCH);

  tmc5240_fieldWrite(motorAddr, TMC5240_LATCH_L_ACTIVE_FIELD, 0); // disable save val when hit left
  tmc5240_fieldWrite(motorAddr, TMC5240_STATUS_LATCH_L_FIELD, 1); // clear latch
  tmc5240_fieldWrite(motorAddr, TMC5240_LATCH_L_INACTIVE_FIELD, 1); // save val when unhit left
  
  // wait for unhit left
  tmc5240_fieldWrite(motorAddr, TMC5240_STOP_L_ENABLE_FIELD, 0); // disable stop left so that keep moving
  while (tmc5240_fieldRead(motorAddr, TMC5240_STATUS_LATCH_L_FIELD) == 0) {
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  vTaskDelay(250 / portTICK_PERIOD_MS); // go a bit further
  tmc5240_writeRegister(motorAddr, TMC5240_VMAX, 0);
  vTaskDelay(250 / portTICK_PERIOD_MS); // stop for a moment
  tmc5240_fieldWrite(motorAddr, TMC5240_LATCH_L_INACTIVE_FIELD, 0); // disable save val when unhit left
  tmc5240_fieldWrite(motorAddr, TMC5240_STATUS_LATCH_L_FIELD, 1); // clear latch

  tmc5240_fieldWrite(motorAddr, TMC5240_SWAP_LR_FIELD, 1); // swap left and right
  tmc5240_fieldWrite(motorAddr, TMC5240_STOP_R_ENABLE_FIELD, 1); // stop when hit right
  tmc5240_fieldWrite(motorAddr, TMC5240_LATCH_R_ACTIVE_FIELD, 1); // save val when hit right
  tmc5240_fieldWrite(motorAddr, TMC5240_RAMPMODE_FIELD, TMC5240_MODE_VELPOS); // will only stop for right in pos dir
  tmc5240_writeRegister(motorAddr, TMC5240_VMAX, 10000);
  while (tmc5240_fieldRead(motorAddr, TMC5240_EVENT_STOP_R_FIELD) == 0) {
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }

  // found right, save it
  vTaskDelay(250 / portTICK_PERIOD_MS); // stop for a moment
  uint32_t right_val = tmc5240_readRegister(motorAddr, TMC5240_XLATCH);

  // stop
  tmc5240_writeRegister(motorAddr, TMC5240_VMAX, 0);
  tmc5240_fieldWrite(motorAddr, TMC5240_LATCH_R_ACTIVE_FIELD, 0); // disable save val when hit left
  tmc5240_fieldWrite(motorAddr, TMC5240_STATUS_LATCH_R_FIELD, 1); // clear latch
  tmc5240_fieldWrite(motorAddr, TMC5240_STOP_R_ENABLE_FIELD, 0); // disable stop right so that can keep moving
  tmc5240_fieldWrite(motorAddr, TMC5240_SWAP_LR_FIELD, 0); // unswap left and right

  tmc5240_writeRegister(motorAddr, TMC5240_XTARGET, right_val);
  tmc5240_fieldWrite(motorAddr, TMC5240_RAMPMODE_FIELD, TMC5240_MODE_POSITION); // pos mode to go to center
  tmc5240_writeRegister(motorAddr, TMC5240_VMAX, 10000);
  tmc5240_fieldWrite(motorAddr, TMC5240_EVENT_POS_REACHED_FIELD, 1); // clear pos reached
  tmc5240_writeRegister(motorAddr, TMC5240_XTARGET, right_val + ((left_val - right_val) / 2));
  while (tmc5240_fieldRead(motorAddr, TMC5240_EVENT_POS_REACHED_FIELD) == 0) {
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  tmc5240_writeRegister(motorAddr, TMC5240_VMAX, 0);
  tmc5240_writeRegister(motorAddr, TMC5240_XACTUAL, 0);
  tmc5240_writeRegister(motorAddr, TMC5240_XTARGET, 0);

  // return total_val;
  return MICROSTEPS_PER_ROTATION_X;
}

uint32_t tmc2540_home_y(TaskMotor_motor_addr_t motorAddr) {
  TaskStatusLED_setState(TASKSTATUSLED_LED_STATE_ORANGE_BLINK_RAPID);
  uint32_t gconf = 0;
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_FAST_STANDSTILL_FIELD, 0);         // 1 Normal time
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_EN_PWM_MODE_FIELD, 0);             // 2 enable StealthChop2
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_SHAFT_FIELD, 0);                   // 4 normal direction
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_DIAG0_STALL_STEP_FIELD, 0);        // 7 DIAG0 out
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_DIAG1_STALL_DIR_FIELD, 0);         // 8 DIAG1 out
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_DIAG0_INT_PUSHPULL_FIELD, 0);      // 12 DIAG0 out active low
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_DIAG1_POSCOMP_PUSHPULL_FIELD, 0);  // 13 DIAG1 out active low
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_SMALL_HYSTERESIS_FIELD, 0);        // 14 hysteresis 1/16
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_STOP_ENABLE_FIELD, 0);             // 15 do not hard stop on enca trigger
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_DIRECT_MODE_FIELD, 0);             // 16 disable direct motor phase control through serial
  gconf = tmc5240_fieldUpdate(gconf, TMC5240_LENGTH_STEP_PULSE_FIELD, 0);       // 17 length step pulse
  tmc5240_writeRegister(motorAddr, TMC5240_GCONF, gconf);

  uint32_t drvconf = 0;
  drvconf = tmc5240_fieldUpdate(drvconf, TMC5240_CURRENT_RANGE_FIELD, 0);       // 0 current range 1A
  drvconf = tmc5240_fieldUpdate(drvconf, TMC5240_CURRENT_RANGE_FIELD, 0b01);    // 0 current range 2A
  drvconf = tmc5240_fieldUpdate(drvconf, TMC5240_SLOPE_CONTROL_FIELD, 0);       // 1 slope control 100V/us
  tmc5240_writeRegister(motorAddr, TMC5240_DRV_CONF, drvconf);

  tmc5240_fieldWrite(motorAddr, TMC5240_GLOBAL_SCALER_FIELD, 190);  // 1.5A

  uint32_t iholdirun = 0;
  iholdirun = tmc5240_fieldUpdate(iholdirun, TMC5240_IHOLD_FIELD, 8);           // 0 ihold
  iholdirun = tmc5240_fieldUpdate(iholdirun, TMC5240_IHOLD_FIELD, 21);          // 0 ihold 70% of irun
  iholdirun = tmc5240_fieldUpdate(iholdirun, TMC5240_IRUN_FIELD, 31);           // 1 irun
  iholdirun = tmc5240_fieldUpdate(iholdirun, TMC5240_IHOLDDELAY_FIELD, 0x1);    // 2 iholddelay power down delay
  iholdirun = tmc5240_fieldUpdate(iholdirun, TMC5240_IRUNDELAY_FIELD, 0x4);     // 3 irundelay duration of motor power up
  tmc5240_writeRegister(motorAddr, TMC5240_IHOLD_IRUN, iholdirun);


  uint32_t chopconf = 0;
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_TOFF_FIELD, 0b0011);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_TFD_ALL_FIELD, 0b101);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_HEND_OFFSET_FIELD, 0b0010);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_FD3_FIELD, 0);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_DISFDCC_FIELD, 0);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_CHM_FIELD, 0); // ?
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_TBL_FIELD, 0x2);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_VHIGHFS_FIELD, 0); // ?
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_VHIGHCHM_FIELD, 0); // ?
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_TPFD_FIELD, 0x4);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_MRES_FIELD, 0);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_INTPOL_FIELD, 0x1);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_DEDGE_FIELD, 0); // reserved
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_DISS2G_FIELD, 0x0);
  chopconf = tmc5240_fieldUpdate(chopconf, TMC5240_DISS2VS_FIELD, 0x0);
  tmc5240_writeRegister(motorAddr, TMC5240_CHOPCONF, chopconf);

  tmc5240_fieldWrite(motorAddr, TMC5240_RAMPMODE_FIELD, TMC5240_MODE_VELPOS);
  tmc5240_writeRegister(motorAddr, TMC5240_AMAX, 3000);
  tmc5240_writeRegister(motorAddr, TMC5240_DMAX, 3000);

  tmc5240_fieldWrite(motorAddr, TMC5240_VSTART_FIELD, 0);                       // start vel
  tmc5240_fieldWrite(motorAddr, TMC5240_VSTOP_FIELD, 100);                      // stop vel DO NOT SET 0 IN POS MODE default 0xA
  tmc5240_fieldWrite(motorAddr, TMC5240_TZEROWAIT_FIELD, 100);                  // wait time to next movement

  // HOME
  // Stop on sg
  tmc5240_fieldWrite(motorAddr, TMC5240_SG_STOP_FIELD, 1); // stop when hit
  tmc5240_fieldWrite(motorAddr, TMC5240_SFILT_FIELD, 0);
  tmc5240_fieldWrite(motorAddr, TMC5240_RAMPMODE_FIELD, TMC5240_MODE_VELNEG);
  tmc5240_fieldWrite(motorAddr, TMC5240_TCOOLTHRS_FIELD, 4999);
  tmc5240_fieldWrite(motorAddr, TMC5240_SGT_FIELD, 5);
  tmc5240_writeRegister(motorAddr, TMC5240_VMAX, 300000);
  tmc5240_writeRegister(motorAddr, TMC5240_VMAX, 100000);
  size_t timeout_ms = 0;
  while (tmc5240_fieldRead(motorAddr, TMC5240_EVENT_STOP_SG_FIELD) == 0 && timeout_ms < 500) {
    vTaskDelay(1 / portTICK_PERIOD_MS);
    timeout_ms++;
  }
  // found left, save it
  tmc5240_writeRegister(motorAddr, TMC5240_VMAX, 0);
  vTaskDelay(250 / portTICK_PERIOD_MS); // stop for a moment
  tmc5240_writeRegister(motorAddr, TMC5240_XACTUAL, MICROSTEPS_HOME_OFFSET_Y);

  tmc5240_fieldWrite(motorAddr, TMC5240_EVENT_STOP_SG_FIELD, 1); // clear stop when hit
  tmc5240_fieldWrite(motorAddr, TMC5240_SG_STOP_FIELD, 0); // disable stop when hit

  // goto 0
  tmc5240_writeRegister(motorAddr, TMC5240_XTARGET, 0);
  tmc5240_fieldWrite(motorAddr, TMC5240_RAMPMODE_FIELD, TMC5240_MODE_POSITION);
  tmc5240_writeRegister(motorAddr, TMC5240_VMAX, 10000);
  while (tmc5240_fieldRead(motorAddr, TMC5240_EVENT_POS_REACHED_FIELD) == 0) {
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  tmc5240_writeRegister(motorAddr, TMC5240_VMAX, 0);
  // return right_val;
  return MICROSTEPS_PER_ROTATION_Y;
}
