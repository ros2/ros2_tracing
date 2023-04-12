^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tracetools_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Enable document generation using rosdoc2 for ament_python pkgs (`#50 <https://github.com/ros2/ros2_tracing/issues/50>`_)
* Contributors: Yadu

5.1.0 (2023-03-02)
------------------

5.0.0 (2023-02-14)
------------------
* Remove deprecated context_names parameter (`#38 <https://github.com/ros2/ros2_tracing/issues/38>`_)
* Contributors: Christophe Bedard

4.0.0 (2022-01-20)
------------------
* Disable kernel tracing by default
* Don't require kernel tracer and detect when it's not installed
* Add support for preloading pthread and dl instrumentation shared libs
* Remove profile_fast option and consider LD_PRELOADing both libs
* Improve event matching for shared lib preloading
* Improve LdPreload action's lib-finding function and add proper tests
* Fix multiple LdPreload actions not working and add test
* Deprecate 'context_names' param and replace with 'context_fields'
* Support per-domain context fields for the Trace action
* Improve LdPreload.get_shared_lib_path() for when a static lib may exist
* Move some tests from tracetools_launch to test_tracetools_launch
* Expose Trace action as frontend action and support substitutions
* Contributors: Christophe Bedard, Ingo Lütkebohle

2.1.0 (2021-01-13)
------------------
* Allow configuring tracing directory through environment variables
* Contributors: Christophe Bedard

1.0.1 (2020-05-27)
------------------
* Deprecation fixes
* Contributors: Christophe Bedard

1.0.0 (2020-04-24)
------------------
* Document what kind of lib_name LdPreload expects
* Add logs for LdPreload action on success or failure
* Contributors: Christophe Bedard

0.3.0 (2020-03-04)
------------------
* Add logs for trace action init and fini
* Expose context enabling through CLI and Trace action
* Extract LdPreload action from Trace action to support preloading any lib
* Contributors: Christophe Bedard

0.2.12 (2019-12-09)
-------------------
* Use imperative mood in constructor docstring.
* Contributors: Christophe Bedard, Steven! Ragnarök

0.2.11 (2019-12-09)
-------------------
* Register Python packages in the ament index
* Contributors: Christophe Bedard

0.2.7 (2019-09-09)
------------------
* Fix missing ament_xmllint dependency
* Contributors: Christophe Bedard

0.2.5 (2019-08-15)
------------------
* Check if OS is Windows when getting shared lib path
* Contributors: Christophe Bedard

0.2.4 (2019-08-14)
------------------
* Add LD_PRELOAD support for profiling in Trace action
* Contributors: Christophe Bedard

0.2.3 (2019-08-05)
------------------
* Add append_timestamp option for trace action
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
* Add tracing integration into launch
* Contributors: Christophe Bedard, Ingo Lütkebohle
