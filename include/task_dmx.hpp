/* task_dmx.hpp
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

#ifndef TASK_DMX_H
#define TASK_DMX_H

typedef uint16_t TaskDMX_address_t;

bool TaskDMX_setupQueue();
void TaskDMX_setAddress(TaskDMX_address_t address);
void TaskDMX(void *pvParameters) __attribute__ ((flatten));

#endif // TASK_DMX_H
