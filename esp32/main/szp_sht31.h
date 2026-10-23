/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
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
 */

// The Lichuang SZP's external air sensor: a Sensirion SHT31 temperature
// and humidity module plugged into the board's I2C expansion port (J1).
// The sampler lives in the muse component (muse_sht31.c) and keeps the
// latest reading; sensors.read answers with it.

#pragma once

#include "cJSON.h"

// sensors.read: the latest temperature and humidity readings, with their
// age. With no recent reading, an error.
cJSON *szp_sht31_command(void);
