^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2trace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

6.3.1 (2023-05-11)
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
