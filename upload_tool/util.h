#pragma once

#include <iostream>

#define ExitOnFail(expr) \
  do {                   \
    if (!(expr))         \
      return false;      \
  } while (0)

#define ExitMsgOnFail(expr, msg)     \
  do {                               \
    if (!(expr)) {                   \
      std::cerr << msg << std::endl; \
      return false;                  \
    }                                \
  } while (0)
