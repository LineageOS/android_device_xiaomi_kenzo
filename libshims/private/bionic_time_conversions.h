#include <time.h>

bool timespec_from_absolute_timespec(timespec& ts, const timespec& abs_ts, clockid_t clock);
