^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tracetools_trace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

8.2.0 (2024-04-16)
------------------
* Replace all occurences of index.ros.org (`#114 <https://github.com/ros2/ros2_tracing/issues/114>`_)
* Contributors: Chris Lalancette

8.1.0 (2024-03-27)
------------------
* Improve tracing configuration error reporting (`#85 <https://github.com/ros2/ros2_tracing/issues/85>`_)
* Add a space in between not and parentheses. (`#88 <https://github.com/ros2/ros2_tracing/issues/88>`_)
* Contributors: Chris Lalancette, Christophe Bedard

8.0.0 (2024-01-23)
------------------
* Switch to custom lttng-ctl Python bindings (`#81 <https://github.com/ros2/ros2_tracing/issues/81>`_)
* Contributors: Christophe Bedard

7.1.0 (2023-08-23)
------------------
* Create start/pause/resume/stop sub-commands for 'ros2 trace' (`#70 <https://github.com/ros2/ros2_tracing/issues/70>`_)
* Contributors: Christophe Bedard

7.0.0 (2023-06-09)
------------------

6.4.1 (2023-05-11)
------------------
* Detect issue with LTTng and Docker and report error when tracing (`#66 <https://github.com/ros2/ros2_tracing/issues/66>`_)
* Contributors: Christophe Bedard

6.4.0 (2023-04-28)
------------------

6.3.0 (2023-04-18)
------------------

6.2.0 (2023-04-18)
------------------
* Error out if trace already exists unless 'append' option is used (`#58 <https://github.com/ros2/ros2_tracing/issues/58>`_)
* Improve 'ros2 trace' command error handling & add end-to-end tests (`#54 <https://github.com/ros2/ros2_tracing/issues/54>`_)
* Make subbuffer size configurable with Trace action (`#51 <https://github.com/ros2/ros2_tracing/issues/51>`_)
* Contributors: Christophe Bedard, Christopher Wecht

6.1.0 (2023-04-13)
------------------
* Add intra-process tracepoints (`#30 <https://github.com/ros2/ros2_tracing/issues/30>`_)
* Contributors: ymski

6.0.0 (2023-04-12)
------------------
* Allow requiring minimum lttng package version for is_lttng_installed (`#59 <https://github.com/ros2/ros2_tracing/issues/59>`_)
* Include tracepoints by default on Linux (`#31 <https://github.com/ros2/ros2_tracing/issues/31>`_)
* Enable document generation using rosdoc2 for ament_python pkgs (`#50 <https://github.com/ros2/ros2_tracing/issues/50>`_)
* Contributors: Christophe Bedard, Yadu

5.1.0 (2023-03-02)
------------------

5.0.0 (2023-02-14)
------------------
* Replace distutils.version.StrictVersion with packaging.version.Version (`#42 <https://github.com/ros2/ros2_tracing/issues/42>`_)
* Remove deprecated context_names parameter (`#38 <https://github.com/ros2/ros2_tracing/issues/38>`_)
* Contributors: Christophe Bedard

4.0.0 (2022-01-20)
------------------
* Disable kernel tracing by default
* Don't require kernel tracer and detect when it's not installed
* Introduce constants for tracepoint names
* Optimize default tracing session channel config values
* Deprecate 'context_names' param and replace with 'context_fields'
* Support per-domain context fields for the Trace action
* Contributors: Christophe Bedard

3.1.0 (2021-08-11)
------------------
* Add support for rmw init/pub, take, and executor tracepoints
* Contributors: Christophe Bedard

2.2.0 (2021-03-29)
------------------
* Add support for rcl_publish and rclcpp_publish tracepoints
* Contributors: Christophe Bedard

2.1.0 (2021-01-13)
------------------
* Fix flake8 blind except error by using more concrete types
* Allow configuring tracing directory through environment variables
* Cleanly stop ros2trace/tracetools_trace tracing on SIGINT
* Add instrumentation support for linking a timer to a node
* Contributors: Christophe Bedard

2.0.0 (2020-10-12)
------------------
* Add lifecycle node state transition instrumentation
* Contributors: Christophe Bedard, Ingo Lütkebohle

1.0.1 (2020-05-27)
------------------
* Fail gracefully when trying to trace if LTTng is not installed
* Contributors: Christophe Bedard

1.0.0 (2020-04-24)
------------------
* Start a session daemon if there isn't one before setting up tracing
* Contributors: Christophe Bedard

0.3.0 (2020-03-04)
------------------
* Add more context types and refactor mapping between name and constant
* Check version of LTTng Python module and raise error if below 2.10.7
* Add logs for trace action init and fini
* Expose context enabling through CLI and Trace action
* Add kmem_mm_page_alloc|free to default kernel events
* Remove sched_waking/sched_wakeup from the default kernel events list
* Contributors: Christophe Bedard

0.2.11 (2019-12-09)
-------------------
* Register Python packages in the ament index
* Contributors: Christophe Bedard

0.2.10 (2019-11-17)
-------------------
* Make printing list of enabled events more readable
* Add new rclcpp_subscription_init tracepoint to default ROS events list
* Contributors: Christophe Bedard

0.2.8 (2019-10-14)
------------------
* Re-order args for trace command
* Contributors: Christophe Bedard

0.2.7 (2019-09-09)
------------------
* Fix missing ament_xmllint dependency
* Contributors: Christophe Bedard

0.2.6 (2019-08-16)
------------------
* Extract lttng interface functions to other files
* Contributors: Christophe Bedard

0.2.5 (2019-08-15)
------------------
* Make lttng interface silently do nothing if lttng cannot be imported
* Contributors: Christophe Bedard

0.2.2 (2019-08-01)
------------------
* Revert "Replace special character to fix encoding issue"
* Contributors: Christophe Bedard

0.2.1 (2019-07-31)
------------------
* Replace special character to fix encoding issue
* Contributors: Christophe Bedard

0.1.0 (2019-07-11)
------------------
* Use proper arg parser and event names completers
* Add tracing utilities
* Contributors: Christophe Bedard, Ingo Lütkebohle
