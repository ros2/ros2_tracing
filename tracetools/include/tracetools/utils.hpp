#ifndef __TRACETOOLS_UTILS_H_
#define __TRACETOOLS_UTILS_H_

#include <stddef.h>

template<typename T, typename... U>
size_t get_address(std::function<T(U...)> f) {
    typedef T(fnType)(U...);
    fnType ** fnPointer = f.template target<fnType*>();
    // Might be a lambda
    if (fnPointer == nullptr) {
        return 0;
    }
    return (size_t)*fnPointer;
}

const char * get_symbol(void * funptr);

#endif /* __TRACETOOLS_UTILS_H_ */
