# ros2_tracing

[![pipeline status](https://gitlab.com/ros-tracing/ros2_tracing/badges/foxy/pipeline.svg)](https://gitlab.com/ros-tracing/ros2_tracing/commits/foxy)
[![codecov](https://codecov.io/gl/ros-tracing/ros2_tracing/branch/foxy/graph/badge.svg)](https://codecov.io/gl/ros-tracing/ros2_tracing)

Tracing tools for ROS 2.

## Building

As of Foxy, these instructions also apply to an installation from the Debian packages; it will not work out-of-the-box. Also, note that tracing using `ros2_tracing` is not supported on non-Linux systems.

If LTTng is not found during build, or if the [`TRACETOOLS_DISABLED` option is enabled](#disabling-tracing), then this package will not do anything.

To enable tracing:

1. Install [LTTng](https://lttng.org/docs/v2.11/) (`>=2.11.1`) with the Python bindings to control tracing and read traces:
    ```
    $ sudo apt-get update
    $ sudo apt-get install lttng-tools lttng-modules-dkms liblttng-ust-dev
    $ sudo apt-get install python3-babeltrace python3-lttng
    ```
2. Build (at least up to `rcl` & `rclcpp`):
    ```
    $ colcon build
    ```
3. Source and check that tracing is enabled:
    ```
    $ source ./install/local_setup.bash
    $ ros2 run tracetools status
    ```

### Disabling tracing

Alternatively, to build and disable tracing, use `TRACETOOLS_DISABLED`:

```
$ colcon build --cmake-args " -DTRACETOOLS_DISABLED=ON"
```

## Tracing

The steps above will not lead to trace data being generated, and thus they will have no impact on execution. LTTng has to be configured for tracing. The packages in this repo provide two options.

### Trace command

The first option is to use the `ros2 trace` command.

```
$ ros2 trace
```

By default, it will enable all ROS tracepoints and a few kernel tracepoints. The trace will be written to `~/.ros/tracing/session-YYYYMMDDHHMMSS`. Run the command with `-h` for more information.

### Launch file trace action

Another option is to use the `Trace` action in a launch file along with your `Node` action(s). This way, tracing happens when launching the launch file.

```
$ ros2 launch tracetools_launch example.launch.py
```

See [this example launch file](./tracetools_launch/launch/example.launch.py) for more information.

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

Package containing system tests for `tracetools` and the tools to support them.

### tracetools_trace

Package containing tools to enable tracing.

## Analysis

See [`tracetools_analysis`](https://gitlab.com/ros-tracing/tracetools_analysis).
