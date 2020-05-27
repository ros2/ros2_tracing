^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tracetools_trace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
