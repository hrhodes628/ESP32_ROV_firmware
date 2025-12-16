#pragma once
#include <stdbool.h>

void arming_init(void);

bool arming_is_armed(void);
void arming_set(bool armed);