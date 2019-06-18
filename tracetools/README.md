# tracetools

## Building

If tracing is not enabled when building, or if LTTng is not found, then this package will not do anything.

To enable tracing:

1. Install [LTTng](https://lttng.org/docs/v2.10/#doc-ubuntu):
    ```
    $ sudo apt-get install lttng-tools lttng-modules-dkms liblttng-ust-dev
    ```
2. Build with the `WITH_LTTNG` flag:
    ```
    $ colcon build --cmake-args " -DWITH_LTTNG=ON"
    ```
3. Check if tracing is enabled (after sourcing):
    ```
    $ ros2 run tracetools tracetools_status
    ```

## Tracing

By default, the steps above will not lead to trace data being generated, and thus they will have no impact on execution.

LTTng has to be enabled: TODO mention scripts
