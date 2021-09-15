# ros2_tracing

[![pipeline status](https://gitlab.com/ros-tracing/ros2_tracing/badges/master/pipeline.svg)](https://gitlab.com/ros-tracing/ros2_tracing/commits/master)
[![codecov](https://codecov.io/gl/ros-tracing/ros2_tracing/branch/master/graph/badge.svg)](https://codecov.io/gl/ros-tracing/ros2_tracing)

Tracing tools for ROS 2.

## Overview

`ros2_tracing` provides [tracing instrumentation](#tracetools) for the core ROS 2 packages.
It also provides [tools to configure tracing](#tracing) through [a launch action](#launch-file-trace-action) and a [`ros2` CLI command](#trace-command).

`ros2_tracing` currently only supports the [LTTng](https://lttng.org/) tracer.
Consequently, it currently only supports Linux.

## Building

As of Foxy, these instructions also apply to an installation from the Debian packages; it will not work out-of-the-box.

If LTTng is not found during build, or if the [`TRACETOOLS_DISABLED` option is enabled](#disabling-tracing), then this package will not do anything.

To enable tracing:

1. Install [LTTng](https://lttng.org/docs/v2.11/) (`>=2.11.1`) with the Python bindings to control tracing and read traces:
    ```
    $ sudo apt-get update
    $ sudo apt-get install lttng-tools lttng-modules-dkms liblttng-ust-dev
    $ sudo apt-get install python3-babeltrace python3-lttng
    ```
2. Build:
    *  If you've already built ROS 2 from source before installing LTTng, you will need to re-build at least up to `tracetools`:
        ```
        $ colcon build --packages-up-to tracetools --cmake-force-configure
        ```
    * If you rely on the ROS 2 binaries (Debian packages, release binaries, or prerelease binaries), you will need to clone this repo into your workspace and build at least up to `tracetools`:
        ```
        $ cd src/
        $ git clone https://gitlab.com/ros-tracing/ros2_tracing.git
        $ cd ../
        $ colcon build --packages-up-to tracetools
        ```
3. Source and check that tracing is enabled:
    ```
    $ source ./install/setup.bash
    $ ros2 run tracetools status
    Tracing enabled
    ```

### Disabling tracing

Alternatively, to build and disable tracing, use `TRACETOOLS_DISABLED`:

```
$ colcon build --cmake-args " -DTRACETOOLS_DISABLED=ON"
```

This will remove all instrumentation from the core ROS 2 packages, and thus they will not depend on or link against the shared library provided by the [`tracetools` package](#tracetools).

## Tracing

The steps above will not lead to trace data being generated, and thus they will have no impact on execution.
LTTng has to be configured for tracing.
The packages in this repo provide two options: a [command](#Trace-command) and a [launch file action](#Launch-file-trace-action).

The tracing directory can be configured using command/launch action parameters, or through environment variables with the following logic:

* Use `$ROS_TRACE_DIR` if `ROS_TRACE_DIR` is set and not empty.
* Otherwise, use `$ROS_HOME/tracing`, using `~/.ros` for `ROS_HOME` if not set or if empty.

Additionally, make sure that the `tracing` group exists and that your user is added to it.

```
# Create group if it doesn't exist
$ sudo groupadd -r tracing
# Add user to the group
$ sudo usermod -aG tracing $USER
```

### Trace command

The first option is to use the `ros2 trace` command.

```
$ ros2 trace
```

**If you encounter an error here, make sure to [add your user to the `tracing` group](#tracing).**

By default, it will enable all ROS tracepoints and a few kernel tracepoints.
The trace will be written to `~/.ros/tracing/session-YYYYMMDDHHMMSS`.
Run the command with `-h` for more information.

### Launch file trace action

Another option is to use the `Trace` action in a launch file along with your `Node` action(s).
This way, tracing happens when launching the launch file.

```
$ ros2 launch tracetools_launch example.launch.py
```

See [this example launch file](./tracetools_launch/launch/example.launch.py) and the [`Trace` action](./tracetools_launch/tracetools_launch/action.py) for more information.

## Design

See the [design document](./doc/design_ros_2.md).

## Packages

### ros2trace

Package containing a `ros2cli` extension to enable tracing.

### tracetools

Library to support instrumenting ROS packages, including core packages.

This package claims to be in the **Quality Level 1** category, see the [Quality Declaration](./tracetools/QUALITY_DECLARATION.md) for more details.

See the [API documentation](https://ros-tracing.gitlab.io/ros2_tracing-api/).

### tracetools_launch

Package containing tools to enable tracing through launch files.

### tracetools_read

Package containing tools to read traces.

### tracetools_test

Package containing tools for tracing-related tests.

### tracetools_trace

Package containing tools to enable tracing.

### test_tracetools

Package containing unit and system tests for `tracetools`.

## Analysis

See [`tracetools_analysis`](https://gitlab.com/ros-tracing/tracetools_analysis).
