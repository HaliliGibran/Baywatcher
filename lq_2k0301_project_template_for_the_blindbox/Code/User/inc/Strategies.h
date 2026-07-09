#ifndef _BAYWATCHER_STRATEGIES_H_
#define _BAYWATCHER_STRATEGIES_H_

#include <stdint.h>

void BayWatcher_Apply_Strategy(uint8_t strategy_id);
const char* BayWatcher_Strategy_Name(uint8_t strategy_id);
uint8_t BayWatcher_Strategy_Count(void);
uint8_t BayWatcher_Strategy_Active(void);

#endif
