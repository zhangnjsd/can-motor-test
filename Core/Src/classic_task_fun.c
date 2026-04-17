#include "main.h"
#include <limits.h>
#include "classic_task_fun.h"

void classic_task(void *argument) {
    j = 0;
    for (;;) {
        j --;
        if (j <= INT_MIN) {
            j = 0;
        }
        osDelay(1);
    }
}