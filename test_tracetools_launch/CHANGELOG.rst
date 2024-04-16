^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_tracetools_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

8.2.0 (2024-04-16)
------------------

8.1.0 (2024-03-27)
------------------
* Improve tracing configuration error reporting (`#85 <https://github.com/ros2/ros2_tracing/issues/85>`_)
* Contributors: Christophe Bedard

8.0.0 (2024-01-23)
------------------

7.1.0 (2023-08-23)
------------------

7.0.0 (2023-06-09)
------------------

6.4.1 (2023-05-11)
------------------

6.4.0 (2023-04-28)
------------------

6.3.0 (2023-04-18)
------------------

6.2.0 (2023-04-18)
------------------
* Error out if trace already exists unless 'append' option is used (`#58 <https://github.com/ros2/ros2_tracing/issues/58>`_)
* Make subbuffer size configurable with Trace action (`#51 <https://github.com/ros2/ros2_tracing/issues/51>`_)
* Contributors: Christophe Bedard, Christopher Wecht

6.1.0 (2023-04-13)
------------------

6.0.0 (2023-04-12)
------------------
* Allow requiring minimum lttng package version for is_lttng_installed (`#59 <https://github.com/ros2/ros2_tracing/issues/59>`_)
* Enable document generation using rosdoc2 for ament_python pkgs (`#50 <https://github.com/ros2/ros2_tracing/issues/50>`_)
* Contributors: Christophe Bedard, Yadu

5.1.0 (2023-03-02)
------------------

4.0.0 (2022-01-20)
------------------
* Add support for preloading pthread and dl instrumentation shared libs
* Remove profile_fast option and consider LD_PRELOADing both libs
* Fix multiple LdPreload actions not working and add test
* Deprecate 'context_names' param and replace with 'context_fields'
* Move some tests from tracetools_launch to test_tracetools_launch
* Contributors: Christophe Bedard, Ingo Lütkebohle
