^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_tracetools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

8.2.0 (2024-04-16)
------------------
* Improve tracetools_test and simplify test_tracetools code (`#109 <https://github.com/ros2/ros2_tracing/issues/109>`_)
* Install test_tracetools_mark_process (`#113 <https://github.com/ros2/ros2_tracing/issues/113>`_)
* Remove unnecessary <string> include (`#111 <https://github.com/ros2/ros2_tracing/issues/111>`_)
* Include <string> in mark_process.cpp (`#110 <https://github.com/ros2/ros2_tracing/issues/110>`_)
* Remove unnecessary print in test (`#108 <https://github.com/ros2/ros2_tracing/issues/108>`_)
* Add test for GenericPublisher/Subscriber (`#97 <https://github.com/ros2/ros2_tracing/issues/97>`_)
* Use lttng_ust_tracef instead of lttng_ust__tracef (`#103 <https://github.com/ros2/ros2_tracing/issues/103>`_)
* Contributors: Christophe Bedard, h-suzuki-isp

8.1.0 (2024-03-27)
------------------
* Use a memcmp for the expected symbol name. (`#100 <https://github.com/ros2/ros2_tracing/issues/100>`_)
* Fix the build on RHEL-9. (`#98 <https://github.com/ros2/ros2_tracing/issues/98>`_)
* Allow tracing tests to be run in parallel with other tests (`#95 <https://github.com/ros2/ros2_tracing/issues/95>`_)
* Fix interference between test_tracetools and ros2lifecycle tests (`#96 <https://github.com/ros2/ros2_tracing/issues/96>`_)
* Make tracing test assert messages more descriptive (`#93 <https://github.com/ros2/ros2_tracing/issues/93>`_)
* Update tests and docs after new rmw_publish timestamp field (`#90 <https://github.com/ros2/ros2_tracing/issues/90>`_)
* Contributors: Chris Lalancette, Christophe Bedard

8.0.0 (2024-01-23)
------------------
* Switch to target_link_libraries in test_tracetools. (`#83 <https://github.com/ros2/ros2_tracing/issues/83>`_)
* Contributors: Chris Lalancette

7.1.0 (2023-08-23)
------------------

7.0.0 (2023-06-09)
------------------

6.4.1 (2023-05-11)
------------------
* Improve test coverage of rclcpp_callback_register in test_tracetools (`#69 <https://github.com/ros2/ros2_tracing/issues/69>`_)
* Disable tracing on Android (`#71 <https://github.com/ros2/ros2_tracing/issues/71>`_)
* Contributors: Christophe Bedard, Przemysław Dąbrowski

6.4.0 (2023-04-28)
------------------

6.3.0 (2023-04-18)
------------------

6.2.0 (2023-04-18)
------------------

6.1.0 (2023-04-13)
------------------
* Add intra-process tracepoints (`#30 <https://github.com/ros2/ros2_tracing/issues/30>`_)
* Contributors: ymski

6.0.0 (2023-04-12)
------------------
* Allow requiring minimum lttng package version for is_lttng_installed (`#59 <https://github.com/ros2/ros2_tracing/issues/59>`_)
* Disable tracing on macOS (`#53 <https://github.com/ros2/ros2_tracing/issues/53>`_)
* Include tracepoints by default on Linux (`#31 <https://github.com/ros2/ros2_tracing/issues/31>`_)
* Contributors: Christophe Bedard

5.1.0 (2023-03-02)
------------------
* Fix memory leak in tracetools::get_symbol() (`#43 <https://github.com/ros2/ros2_tracing/issues/43>`_)
* Contributors: Christophe Bedard

5.0.0 (2023-02-14)
------------------
* Update tracing to C++17. (`#33 <https://github.com/ros2/ros2_tracing/issues/33>`_)
* Contributors: Chris Lalancette

4.0.0 (2022-01-20)
------------------
* Introduce constants for tracepoint names
* Move actual tests out of tracetools_test to new test_tracetools package
* Contributors: Christophe Bedard
