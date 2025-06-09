#ifndef INCLUDE_ORDER_FIX_H
#define INCLUDE_ORDER_FIX_H

/*
 * This file ensures the correct include order to avoid type conflicts
 * between algorithm_ctrl.h and exppack_ctrl.h
 */

// First include the core types that both files need
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// Include algorithm_ctrl.h first as it has the primary definitions
#include "algorithm_ctrl.h"

// Then include the EMG controller
#include "robot_emg_controller.h"

#endif /* INCLUDE_ORDER_FIX_H */ 