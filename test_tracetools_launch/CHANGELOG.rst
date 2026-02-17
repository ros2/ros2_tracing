^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_tracetools_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.1.2 (2026-02-16)
------------------
* fix setuptools deprecation (backport `#189 <https://github.com/ros2/ros2_tracing/issues/189>`_) (`#204 <https://github.com/ros2/ros2_tracing/issues/204>`_)
* Contributors: mergify[bot]

4.1.1 (2022-11-07)
------------------
* Merge branch 'clalancette/release-4.1.0' into 'master'
* Contributors: Christophe Bedard

4.0.0 (2022-01-20)
------------------
* Add support for preloading pthread and dl instrumentation shared libs
* Remove profile_fast option and consider LD_PRELOADing both libs
* Fix multiple LdPreload actions not working and add test
* Deprecate 'context_names' param and replace with 'context_fields'
* Move some tests from tracetools_launch to test_tracetools_launch
* Contributors: Christophe Bedard, Ingo Lütkebohle
