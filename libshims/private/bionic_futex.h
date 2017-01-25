static inline __always_inline int __futex(volatile void* ftx, int op, int value, const struct timespec* timeout) {
  // Our generated syscall assembler sets errno, but our callers (pthread functions) don't want to.
  int saved_errno = errno;
  int result = syscall(__NR_futex, ftx, op, value, timeout);
  if (__predict_false(result == -1)) {
    result = -errno;
    errno = saved_errno;
  }
  return result;
}

static inline int __futex_wait_ex(volatile void* ftx, bool shared, int value, const struct timespec* timeout) {
  return __futex(ftx, shared ? FUTEX_WAIT : FUTEX_WAIT_PRIVATE, value, timeout);
}
