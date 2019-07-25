# tracetools

Tracing tools for ROS 2.

## Building

If the `TRACETOOLS_DISABLED` option is enabled during build or if LTTng is not found, then this package will not do anything.

To enable tracing:

1. Install [LTTng](https://lttng.org/docs/#doc-ubuntu) with the Python bindings to control tracing and read traces:
    ```
    $ sudo apt-add-repository ppa:lttng/stable-2.10
    $ sudo apt-get update
    $ sudo apt-get install lttng-tools lttng-modules-dkms liblttng-ust-dev
    $ sudo apt-get install python3-babeltrace python3-lttng
    ```
    Note: the LTTng stable 2.10 PPA is used to get newer versions of the packages.
2. Build
    ```
    $ colcon build
    ```
3. Source and check that tracing is enabled:
    ```
    $ source ./install/local_setup.bash
    $ ros2 run tracetools tracetools_status
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

Another option is to use the `Trace` action in a launch file along with your `Node` action(s).

See [this example launch file](./tracetools_launch/launch/example.launch.py) for more information.
