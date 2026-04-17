#include "main.h"
#include <limits.h>
#include "gimble_task_fun.h"

void gimble_task(void *argument) {
    i = 0;
    for (;;) {
        i ++;
        if (i >= INT_MAX) {
            i = 0;
        }
        osDelay(1);
    }
}