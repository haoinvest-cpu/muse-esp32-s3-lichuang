/*
 * Compat shim (grafted 2026-10-30): the board file registers the camera
 * through camera_gc0308() with a camera_gc0308_config_t; serve that same
 * entry point with the DVP backend (camera_dvp.c) instead of the original
 * camera_gc0308.c. The board file and its config struct stay untouched.
 *
 * camera_dvp() keeps a pointer to its config rather than copying it, and
 * the board passes a stack-local config, so the power callback is copied
 * into this static struct whose lifetime is the whole run.
 */

#include "camera_gc0308.h"
#include "camera_dvp.h"

static camera_dvp_config_t s_dvp_cfg;

const camera_driver_t *camera_gc0308(const camera_gc0308_config_t *config)
{
    s_dvp_cfg.power = config ? config->power : NULL;
    return camera_dvp(&s_dvp_cfg);
}
