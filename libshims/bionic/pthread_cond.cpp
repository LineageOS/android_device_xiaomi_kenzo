#include <pthread.h>

#include <sys/syscall.h>
#include <sys/mman.h>
#include <linux/futex.h>
#include <errno.h>
#include <stdatomic.h>
#include <time.h>
#include <unistd.h>

#include "private/bionic_futex.h"
#include "private/bionic_time_conversions.h"

#define COND_SHARED_MASK 0x0001
#define COND_CLOCK_MASK 0x0002

#define COND_IS_SHARED(c) (((c) & COND_SHARED_MASK) != 0)
#define COND_GET_CLOCK(c) (((c) & COND_CLOCK_MASK) >> 1)

struct pthread_cond_internal_t {
  atomic_uint state;

  bool process_shared() {
    return COND_IS_SHARED(atomic_load_explicit(&state, memory_order_relaxed));
  }

  int get_clock() {
    return COND_GET_CLOCK(atomic_load_explicit(&state, memory_order_relaxed));
  }

#if defined(__LP64__)
  char __reserved[44];
#endif
};

static pthread_cond_internal_t* __get_internal_cond(pthread_cond_t* cond_interface) {
  return reinterpret_cast<pthread_cond_internal_t*>(cond_interface);
}

static int __pthread_cond_timedwait_relative(pthread_cond_internal_t* cond, pthread_mutex_t* mutex,
                                             const timespec* rel_timeout_or_null) {
  unsigned int old_state = atomic_load_explicit(&cond->state, memory_order_relaxed);

  pthread_mutex_unlock(mutex);
  int status = __futex_wait_ex(&cond->state, cond->process_shared(), old_state, rel_timeout_or_null);
  pthread_mutex_lock(mutex);

  if (status == -ETIMEDOUT) {
    return ETIMEDOUT;
  }
  return 0;
}

static int __pthread_cond_timedwait(pthread_cond_internal_t* cond, pthread_mutex_t* mutex,
                                    const timespec* abs_timeout_or_null, clockid_t clock) {
  timespec ts;
  timespec* rel_timeout = NULL;

  if (abs_timeout_or_null != NULL) {
    rel_timeout = &ts;
    if (!timespec_from_absolute_timespec(*rel_timeout, *abs_timeout_or_null, clock)) {
      return ETIMEDOUT;
    }
  }

  return __pthread_cond_timedwait_relative(cond, mutex, rel_timeout);
}

int pthread_cond_timedwait(pthread_cond_t *cond_interface, pthread_mutex_t * mutex,
                           const timespec *abstime) {

  pthread_cond_internal_t* cond = __get_internal_cond(cond_interface);
  return __pthread_cond_timedwait(cond, mutex, abstime, cond->get_clock());
}

int pthread_cond_wait(pthread_cond_t* cond_interface, pthread_mutex_t* mutex) {
  pthread_cond_internal_t* cond = __get_internal_cond(cond_interface);
  return __pthread_cond_timedwait(cond, mutex, NULL, cond->get_clock());
}
