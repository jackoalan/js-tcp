#pragma once
#include <time.h>

constexpr void timespec_add(timespec &result, const timespec &x,
                            const timespec &y) {
  result.tv_sec = x.tv_sec + y.tv_sec;
  result.tv_nsec = x.tv_nsec + y.tv_nsec;

  if (result.tv_nsec >= 1000000000) {
    result.tv_sec += result.tv_nsec / 1000000000;
    result.tv_nsec = result.tv_nsec % 1000000000;
  }
}

constexpr bool timespec_subtract(timespec &result, const timespec &x,
                                 timespec &y) {
  /* Perform the carry for the later subtraction by updating y. */
  using nsec_t = decltype(timespec::tv_nsec);
  if (x.tv_nsec < y.tv_nsec) {
    nsec_t nsec = (y.tv_nsec - x.tv_nsec) / 1000000000 + 1;
    y.tv_nsec -= 1000000000 * nsec;
    y.tv_sec += nsec;
  }
  if (x.tv_nsec - y.tv_nsec > 1000000000) {
    nsec_t nsec = (x.tv_nsec - y.tv_nsec) / 1000000000;
    y.tv_nsec += 1000000000 * nsec;
    y.tv_sec -= nsec;
  }

  /* Compute the time remaining to wait.
     tv_usec is certainly positive. */
  result.tv_sec = x.tv_sec - y.tv_sec;
  result.tv_nsec = x.tv_nsec - y.tv_nsec;

  /* Return true if result is negative. */
  return x.tv_sec < y.tv_sec;
}
