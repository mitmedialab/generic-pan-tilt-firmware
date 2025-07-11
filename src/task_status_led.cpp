/* task_status_led.cpp
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
#include "task_board_io.hpp"
#include "task_status_led.hpp"

// Rate of rapid blink in milliseconds
static const uint16_t TASKSTATUSLED_LED_STATE_BLINK_RAPID_MS = 200;

// Rate of slow blink determined by multiple of
// TASKSTATUSLED_LED_STATE_BLINK_RAPID_MS millisecond value
static const uint8_t TASKSTATUSLED_LED_STATE_BLINK_SLOW_MULT = 5;

static QueueHandle_t TaskStatusLED_queue;

typedef enum e_TaskStatusLED_LED_color_t {
  TASKSTATUSLED_LED_COLOR_GREEN,
  TASKSTATUSLED_LED_COLOR_RED,
  TASKSTATUSLED_LED_COLOR_ORANGE
} TaskStatusLED_LED_color_t;


static void pulseLedNum(const uint8_t pulses, const TaskStatusLED_LED_color_t color);
static inline void external_led_apply();
static inline void external_led_off();
static inline void external_led_green();
static inline void external_led_red();
static inline void external_led_orange();
static inline void external_led_green_toggle();
static inline void external_led_red_toggle();
static inline void external_led_orange_toggle();
static inline void external_led_green_red_toggle();
static inline void external_led_green_orange_toggle();
static inline void external_led_red_orange_toggle();

bool TaskStatusLED_setupQueue() {
  // Queue size is 1 so that most current LED status is always shown
  // This requires the use of xQueueOverwrite()
  const uint8_t QUEUE_SIZE = 1;
  TaskStatusLED_queue = xQueueCreate(QUEUE_SIZE, sizeof(TaskStatusLED_LED_state_t));
  if (TaskStatusLED_queue == 0) {
    return false;
  } else {
    return true;
  }
}

void TaskStatusLED_setState(TaskStatusLED_LED_state_t state) {
  xQueueOverwrite(TaskStatusLED_queue, (void *)&state);
}

void TaskStatusLED(void *pvParameters) {
  (void)pvParameters;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  TaskBoardIO_mcpPinMode(8, OUTPUT);
  TaskBoardIO_mcpPinMode(9, OUTPUT);
  external_led_off();
  static TaskStatusLED_LED_state_t currentLedState = TASKSTATUSLED_LED_STATE_OFF;
  static uint8_t counterBlinkSlow = 0;
  while (1) {
    xQueueReceive(TaskStatusLED_queue, (void *)&currentLedState, 0);

    switch (currentLedState) {
      case TASKSTATUSLED_LED_STATE_OFF:
        external_led_off();
        break;

      case TASKSTATUSLED_LED_STATE_GREEN_ON:
        external_led_green();
        break;

      case TASKSTATUSLED_LED_STATE_RED_ON:
        external_led_red();
        break;

      case TASKSTATUSLED_LED_STATE_ORANGE_ON:
        external_led_orange();
        break;

      case TASKSTATUSLED_LED_STATE_GREEN_BLINK_SLOW:
      case TASKSTATUSLED_LED_STATE_RED_BLINK_SLOW:
      case TASKSTATUSLED_LED_STATE_ORANGE_BLINK_SLOW:
      case TASKSTATUSLED_LED_STATE_GREEN_RED_BLINK_SLOW:
      case TASKSTATUSLED_LED_STATE_GREEN_ORANGE_BLINK_SLOW:
      case TASKSTATUSLED_LED_STATE_RED_ORANGE_BLINK_SLOW:
        if (counterBlinkSlow == 0) {
          switch (currentLedState) {
            case TASKSTATUSLED_LED_STATE_GREEN_BLINK_SLOW:
              external_led_green_toggle();
              break;
            case TASKSTATUSLED_LED_STATE_RED_BLINK_SLOW:
              external_led_red_toggle();
              break;
            case TASKSTATUSLED_LED_STATE_ORANGE_BLINK_SLOW:
              external_led_orange_toggle();
              break;
            case TASKSTATUSLED_LED_STATE_GREEN_RED_BLINK_SLOW:
              external_led_green_red_toggle();
              break;
            case TASKSTATUSLED_LED_STATE_GREEN_ORANGE_BLINK_SLOW:
              external_led_green_orange_toggle();
              break;
            case TASKSTATUSLED_LED_STATE_RED_ORANGE_BLINK_SLOW:
              external_led_red_orange_toggle();
              break;
            default:
              // Catch error state as off
              currentLedState = TASKSTATUSLED_LED_STATE_OFF;
              external_led_off();
              break;
          }
        }
        counterBlinkSlow++;
        if (counterBlinkSlow >= TASKSTATUSLED_LED_STATE_BLINK_SLOW_MULT) {
          counterBlinkSlow = 0;
        }
        break;

      case TASKSTATUSLED_LED_STATE_GREEN_BLINK_RAPID:
      case TASKSTATUSLED_LED_STATE_RED_BLINK_RAPID:
      case TASKSTATUSLED_LED_STATE_ORANGE_BLINK_RAPID:
      case TASKSTATUSLED_LED_STATE_GREEN_RED_BLINK_RAPID:
      case TASKSTATUSLED_LED_STATE_GREEN_ORANGE_BLINK_RAPID:
      case TASKSTATUSLED_LED_STATE_RED_ORANGE_BLINK_RAPID:
        switch (currentLedState) {
          case TASKSTATUSLED_LED_STATE_GREEN_BLINK_RAPID:
            external_led_green_toggle();
            break;
          case TASKSTATUSLED_LED_STATE_RED_BLINK_RAPID:
            external_led_red_toggle();
            break;
          case TASKSTATUSLED_LED_STATE_ORANGE_BLINK_RAPID:
            external_led_orange_toggle();
            break;
          case TASKSTATUSLED_LED_STATE_GREEN_RED_BLINK_RAPID:
            external_led_green_red_toggle();
            break;
          case TASKSTATUSLED_LED_STATE_GREEN_ORANGE_BLINK_RAPID:
            external_led_green_orange_toggle();
            break;
          case TASKSTATUSLED_LED_STATE_RED_ORANGE_BLINK_RAPID:
            external_led_red_orange_toggle();
            break;
          default:
            // Catch error state as off
            currentLedState = TASKSTATUSLED_LED_STATE_OFF;
            external_led_off();
            break;
        }
        break;

      case TASKSTATUSLED_LED_STATE_GREEN_PULSE_2:
        pulseLedNum(2, TASKSTATUSLED_LED_COLOR_GREEN);
        break;
      case TASKSTATUSLED_LED_STATE_RED_PULSE_2:
        pulseLedNum(2, TASKSTATUSLED_LED_COLOR_RED);
        break;
      case TASKSTATUSLED_LED_STATE_ORANGE_PULSE_2:
        pulseLedNum(2, TASKSTATUSLED_LED_COLOR_ORANGE);
        break;

      case TASKSTATUSLED_LED_STATE_GREEN_PULSE_3:
        pulseLedNum(3, TASKSTATUSLED_LED_COLOR_GREEN);
        break;
      case TASKSTATUSLED_LED_STATE_RED_PULSE_3:
        pulseLedNum(3, TASKSTATUSLED_LED_COLOR_RED);
        break;
      case TASKSTATUSLED_LED_STATE_ORANGE_PULSE_3:
        pulseLedNum(3, TASKSTATUSLED_LED_COLOR_ORANGE);
        break;

      case TASKSTATUSLED_LED_STATE_GREEN_PULSE_4:
        pulseLedNum(4, TASKSTATUSLED_LED_COLOR_GREEN);
        break;
      case TASKSTATUSLED_LED_STATE_RED_PULSE_4:
        pulseLedNum(4, TASKSTATUSLED_LED_COLOR_RED);
        break;
      case TASKSTATUSLED_LED_STATE_ORANGE_PULSE_4:
        pulseLedNum(4, TASKSTATUSLED_LED_COLOR_ORANGE);
        break;

      default:
        // Catch error state as off
        currentLedState = TASKSTATUSLED_LED_STATE_OFF;
        external_led_off();
        break;
    }
    external_led_apply();
    vTaskDelayUntil(&xLastWakeTime, (TASKSTATUSLED_LED_STATE_BLINK_RAPID_MS / portTICK_PERIOD_MS));
  }
}

static void pulseLedNum(const uint8_t pulses, const TaskStatusLED_LED_color_t color) {
  static uint8_t counterBlinkSpecific = 0;
  const uint8_t blink_duration = pulses * 2;
  const uint8_t total_duration = blink_duration * 2;
  if (counterBlinkSpecific >= total_duration - 1) {
    counterBlinkSpecific = 0;
  }
  if (counterBlinkSpecific % 2 == 0 && counterBlinkSpecific <= blink_duration - 1) {
    switch (color) {
      case TASKSTATUSLED_LED_COLOR_GREEN:
        external_led_green();
        break;
      case TASKSTATUSLED_LED_COLOR_RED:
        external_led_red();
        break;
      case TASKSTATUSLED_LED_COLOR_ORANGE:
        external_led_orange();
        break;
      default:
        external_led_off();
        break;
    }
  } else {
    external_led_off();
  }
  counterBlinkSpecific++;
}

static PinStatus board_io_green_led_state = HIGH;
static PinStatus board_io_red_led_state = HIGH;
static inline void external_led_apply() {
  TaskBoardIO_mcpDigitalWrite(8, board_io_red_led_state);
  TaskBoardIO_mcpDigitalWrite(9, board_io_green_led_state);
}
static inline void external_led_off() {
  board_io_green_led_state = HIGH;
  board_io_red_led_state = HIGH;
}
static inline void external_led_green() {
  board_io_green_led_state = LOW;
  board_io_red_led_state = HIGH;
}
static inline void external_led_red() {
  board_io_green_led_state = HIGH;
  board_io_red_led_state = LOW;
}
static inline void external_led_orange() {
  board_io_green_led_state = LOW;
  board_io_red_led_state = LOW;
}
static inline void external_led_green_toggle() {
  switch (board_io_green_led_state) {
    case HIGH:
      external_led_green();
      break;

    case LOW:
    default:
      external_led_off();
      break;
  }
}
static inline void external_led_red_toggle() {
  switch (board_io_red_led_state) {
    case HIGH:
      external_led_red();
      break;

    case LOW:
    default:
      external_led_off();
      break;
  }
}
static inline void external_led_orange_toggle() {
  switch (board_io_red_led_state) {
    case HIGH:
      external_led_orange();
      break;

    case LOW:
    default:
      external_led_off();
      break;
  }
}
static inline void external_led_green_red_toggle() {
  switch (board_io_green_led_state) {
    case HIGH:
      external_led_green();
      break;

    case LOW:
    default:
      external_led_red();
      break;
  }
}
static inline void external_led_green_orange_toggle() {
  switch (board_io_red_led_state) {
    case HIGH:
      external_led_orange();
      break;

    case LOW:
    default:
      external_led_green();
      break;
  }
}
static inline void external_led_red_orange_toggle() {
  switch (board_io_green_led_state) {
    case HIGH:
      external_led_orange();
      break;

    case LOW:
    default:
      external_led_red();
      break;
  }
}
