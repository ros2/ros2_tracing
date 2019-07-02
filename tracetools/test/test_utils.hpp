#include <memory>
#include <type_traits>

#include "tracetools/utils.hpp"
#include "function_traits.hpp"


template<typename ParamT>
class SomeGenericClass
{
using SharedPtrCallback = std::function<void (const std::shared_ptr<ParamT>)>;
using UniquePtrCallback = std::function<void (const std::unique_ptr<ParamT>)>;

public:
  SomeGenericClass()
  : my_callback_shared_(nullptr), my_callback_unique_(nullptr)
  {}

  template<
    typename TheType,
    typename std::enable_if<
      rclcpp::function_traits::same_arguments<
        TheType,
        SharedPtrCallback
      >::value
    >::type * = nullptr
  >
  void set(TheType callback)
  {
    my_callback_shared_ = callback;
  }

  template<
    typename TheType,
    typename std::enable_if<
      rclcpp::function_traits::same_arguments<
        TheType,
        UniquePtrCallback
      >::value
    >::type * = nullptr
  >
  void set(TheType callback)
  {
    my_callback_unique_ = callback;
  }

  void * get_address_()
  {
    if (my_callback_shared_) {
      return get_address(my_callback_shared_);
    } else if (my_callback_unique_) {
      return get_address(my_callback_unique_);
    } else {
      return (void *)0;
    }
  }

  const char * get_symbol_()
  {
    return get_symbol(get_address_());
  }

private:
  SharedPtrCallback my_callback_shared_;
  UniquePtrCallback my_callback_unique_;
};
