#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <cmath>

#include "LQ_ATIM_PWM.hpp"
#include "LQ_GTIM_PWM.hpp"
#include "LQ_HW_GPIO.hpp"
#include "LQ_TFT18_dri.hpp"

#include "Menu.h"
#include "Motor.h"
#include "ESC.h"
#include "Key.h"

extern BayWatcher_Menu& menu;
extern BayWatcher_Motor motor_sys;
extern BayWatcher_ESC esc_sys;
extern BayWatcher_Key key_sys;

