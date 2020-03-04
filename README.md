# ros2_tracing

[![pipeline status](https://gitlab.com/micro-ROS/ros_tracing/ros2_tracing/badges/master/pipeline.svg)](https://gitlab.com/micro-ROS/ros_tracing/ros2_tracing/commits/master)

Tracing tools for ROS 2.

## Building

If the `TRACETOOLS_DISABLED` option is enabled during build or if LTTng is not found, then this package will not do anything.

To enable tracing:

1. Install [LTTng](https://lttng.org/docs/v2.10/#doc-ubuntu) with the Python bindings to control tracing and read traces:
    ```
    $ sudo apt-add-repository ppa:lttng/stable-2.10
    $ sudo apt-get update
    $ sudo apt-get install lttng-tools lttng-modules-dkms liblttng-ust-dev
    $ sudo apt-get install python3-babeltrace python3-lttng
    ```
    Notes:
    * **Version 2.10.7 or above is required for LTTng**. Recommended: `>=2.10.10` or `>=2.11.1`. Check version with `lttng --version`.  
       On Ubuntu, use the above PPA. On other distros, [build from source](https://lttng.org/docs/v2.10/#doc-building-from-source), making sure to also [build the Python bindings](https://github.com/lttng/lttng-tools#building).

2. Build
    ```
    $ colcon build
    ```
3. Source and check that tracing is enabled:
    ```
    $ source ./install/local_setup.bash
    $ ros2 run tracetools status
    ```

## Tracing

The steps above will not lead to trace data being generated, and thus they will have no impact on execution. LTTng has to be enabled. The packages in this repo provide two options.

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

## Analysis

See [`tracetools_analysis`](https://gitlab.com/micro-ROS/ros_tracing/tracetools_analysis).
