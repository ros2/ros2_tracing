^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2trace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

8.2.0 (2024-04-16)
------------------
* Replace all occurences of index.ros.org (`#114 <https://github.com/ros2/ros2_tracing/issues/114>`_)
* Contributors: Chris Lalancette

8.1.0 (2024-03-27)
------------------

8.0.0 (2024-01-23)
------------------

7.1.0 (2023-08-23)
------------------
* Create start/pause/resume/stop sub-commands for 'ros2 trace' (`#70 <https://github.com/ros2/ros2_tracing/issues/70>`_)
* Contributors: Christophe Bedard

7.0.0 (2023-06-09)
------------------

6.4.1 (2023-05-11)
------------------
* Switch <depend> to <exec_depend> in pure Python packages (`#67 <https://github.com/ros2/ros2_tracing/issues/67>`_)
* Contributors: Christophe Bedard

6.4.0 (2023-04-28)
------------------

6.3.0 (2023-04-18)
------------------
* Move ros2trace tests to new test_ros2trace package (`#63 <https://github.com/ros2/ros2_tracing/issues/63>`_)
* Contributors: Christophe Bedard

6.2.0 (2023-04-18)
------------------
* Error out if trace already exists unless 'append' option is used (`#58 <https://github.com/ros2/ros2_tracing/issues/58>`_)
* Improve 'ros2 trace' command error handling & add end-to-end tests (`#54 <https://github.com/ros2/ros2_tracing/issues/54>`_)
* Contributors: Christophe Bedard

6.1.0 (2023-04-13)
------------------

6.0.0 (2023-04-12)
------------------

5.1.0 (2023-03-02)
------------------

4.1.0 (2022-03-29)
------------------
* Fix 'ros2 trace' fini() error
* Contributors: Christophe Bedard

4.0.0 (2022-01-20)
------------------
* Don't require kernel tracer and detect when it's not installed
* Deprecate 'context_names' param and replace with 'context_fields'
* Contributors: Christophe Bedard

0.3.0 (2020-03-04)
------------------
* Expose context enabling through CLI and Trace action
* Contributors: Christophe Bedard

0.2.11 (2019-12-09)
-------------------
* Remove duplicated code for trace command
* Contributors: Christophe Bedard

0.2.10 (2019-11-17)
-------------------
* Make printing list of enabled events more readable
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
* Add ros2cli extension that wraps tracetools_trace
* Contributors: Christophe Bedard, Ingo Lütkebohle
