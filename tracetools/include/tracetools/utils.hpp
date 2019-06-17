#ifndef TRACETOOLS__UTILS_HPP_
#define TRACETOOLS__UTILS_HPP_

#include <stddef.h>
#include <functional>

template<typename T, typename ... U>
void * get_address(std::function<T(U...)> f)
{
  typedef T (fnType)(U...);
  fnType ** fnPointer = f.template target<fnType *>();
  // Might be a lambda
  if (fnPointer == nullptr) {
    return 0;
  }
  return (void *)*fnPointer;
}

const char * get_symbol(void * funptr);

#endif  // TRACETOOLS__UTILS_HPP_
