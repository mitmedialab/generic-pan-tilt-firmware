/* task_status_led.hpp
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

#ifndef TASK_STATUS_LED_H
#define TASK_STATUS_LED_H

typedef enum e_TaskStatusLED_LED_state_t {
  TASKSTATUSLED_LED_STATE_OFF,
  TASKSTATUSLED_LED_STATE_GREEN_ON,
  TASKSTATUSLED_LED_STATE_GREEN_BLINK_SLOW,
  TASKSTATUSLED_LED_STATE_GREEN_BLINK_RAPID,
  TASKSTATUSLED_LED_STATE_GREEN_PULSE_2,
  TASKSTATUSLED_LED_STATE_GREEN_PULSE_3,
  TASKSTATUSLED_LED_STATE_GREEN_PULSE_4,
  TASKSTATUSLED_LED_STATE_RED_ON,
  TASKSTATUSLED_LED_STATE_RED_BLINK_SLOW,
  TASKSTATUSLED_LED_STATE_RED_BLINK_RAPID,
  TASKSTATUSLED_LED_STATE_RED_PULSE_2,
  TASKSTATUSLED_LED_STATE_RED_PULSE_3,
  TASKSTATUSLED_LED_STATE_RED_PULSE_4,
  TASKSTATUSLED_LED_STATE_ORANGE_ON,
  TASKSTATUSLED_LED_STATE_ORANGE_BLINK_SLOW,
  TASKSTATUSLED_LED_STATE_ORANGE_BLINK_RAPID,
  TASKSTATUSLED_LED_STATE_ORANGE_PULSE_2,
  TASKSTATUSLED_LED_STATE_ORANGE_PULSE_3,
  TASKSTATUSLED_LED_STATE_ORANGE_PULSE_4,
  TASKSTATUSLED_LED_STATE_GREEN_RED_BLINK_SLOW,
  TASKSTATUSLED_LED_STATE_GREEN_RED_BLINK_RAPID,
  TASKSTATUSLED_LED_STATE_GREEN_ORANGE_BLINK_SLOW,
  TASKSTATUSLED_LED_STATE_GREEN_ORANGE_BLINK_RAPID,
  TASKSTATUSLED_LED_STATE_RED_ORANGE_BLINK_SLOW,
  TASKSTATUSLED_LED_STATE_RED_ORANGE_BLINK_RAPID,
} TaskStatusLED_LED_state_t;

bool TaskStatusLED_setupQueue();
void TaskStatusLED_setState(TaskStatusLED_LED_state_t state);
void TaskStatusLED(void *pvParameters) __attribute__ ((flatten));

#endif // TASK_STATUS_LED_H
