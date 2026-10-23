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

// sensors.read glue for the Lichuang SZP's external SHT31 module. The
// sampler task and the cached reading live in the muse component
// (muse_sht31.c, started by the board file); this only formats the
// latest reading the way sensors.read answers everywhere else: each
// reading with its value, unit and age in seconds.

#include "szp_sht31.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "muse_sht31.h"

static double round1(float v)
{
    return round((double)v * 10.0) / 10.0;
}

static void add_reading(cJSON *payload, const char *name, float value,
                        const char *unit, uint32_t age_s)
{
    cJSON *sensor = cJSON_AddObjectToObject(payload, name);
    if (!sensor) {
        return;
    }
    cJSON_AddNumberToObject(sensor, "value", round1(value));
    cJSON_AddStringToObject(sensor, "unit", unit);
    cJSON_AddNumberToObject(sensor, "age_s", (double)age_s);
}

cJSON *szp_sht31_command(void)
{
    float temp_c, rh;
    uint32_t age_s;
    bool ok = muse_sht31_get_ex(&temp_c, &rh, &age_s);

    cJSON *result = cJSON_CreateObject();
    if (!result) {
        return NULL;
    }
    if (!ok) {
        cJSON_AddBoolToObject(result, "ok", false);
        cJSON *error = cJSON_AddObjectToObject(result, "error");
        if (error) {
            cJSON_AddStringToObject(error, "code", "no_readings");
            cJSON_AddStringToObject(error, "message",
                                    "no sensor readings; the SHT31 may be missing");
        }
        return result;
    }
    cJSON *payload = cJSON_CreateObject();
    if (!payload) {
        cJSON_Delete(result);
        return NULL;
    }
    add_reading(payload, "temperature", temp_c, "celsius", age_s);
    add_reading(payload, "humidity", rh, "percent_rh", age_s);
    cJSON_AddBoolToObject(result, "ok", true);
    cJSON_AddItemToObject(result, "payload", payload);
    return result;
}
