#pragma once

#define ExitOnFail(expr) \
  do {                   \
    if (!(expr))         \
      return false;      \
  } while (0)
