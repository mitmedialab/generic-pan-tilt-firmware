/* main.cpp
 *
 * Created by Perry Naseck on 2024-08-24.
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

#include <FreeRTOS.h>
#include <task.h>
#include <Wire.h>
#include "task_board_io.hpp"
#include "task_dmx.hpp"
#include "task_motor.hpp"
#include "task_status_led.hpp"

void setup() {
  Serial1.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial1.println("starting");
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
  TaskBoardIO_setupQueue();
  TaskDMX_setupQueue();
  TaskMotor_setupQueue();
  TaskStatusLED_setupQueue();
  xTaskCreate(TaskBoardIO, "TaskStatusLED", 128, nullptr, 4, nullptr);
  xTaskCreate(TaskDMX, "TaskDMX", 128, nullptr, 1, nullptr);
  xTaskCreate(TaskMotor, "TaskMotor", 128, nullptr, 2, nullptr);
  xTaskCreate(TaskStatusLED, "TaskStatusLED", 128, nullptr, 3, nullptr);
}

void loop() {
  // pass
}
