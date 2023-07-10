# ros2_tracing

[![GitHub CI](https://github.com/ros2/ros2_tracing/actions/workflows/test.yml/badge.svg?branch=rolling)](https://github.com/ros2/ros2_tracing/actions/workflows/test.yml)
[![codecov](https://codecov.io/gh/ros2/ros2_tracing/branch/rolling/graph/badge.svg)](https://codecov.io/gh/ros2/ros2_tracing)

Tracing tools for ROS 2.

## Overview

`ros2_tracing` provides [tracing instrumentation](#tracetools) for the core ROS 2 packages.
It also provides [tools to configure tracing](#tracing) through [a launch action](#launch-file-trace-action) and a [`ros2` CLI command](#trace-command).

`ros2_tracing` currently only supports the [LTTng](https://lttng.org/) tracer.
Consequently, it currently only supports Linux.

**Note**: make sure to use the right branch, depending on the ROS 2 distro: [use `rolling` for Rolling, `galactic` for Galactic, etc.](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Developer-Guide.html#branches)

## Publications & presentations

[Read the `ros2_tracing` paper!](https://arxiv.org/abs/2201.00393)
If you use or refer to `ros2_tracing`, please cite:
* C. Bédard, I. Lütkebohle, and M. Dagenais, ["ros2_tracing: Multipurpose Low-Overhead Framework for Real-Time Tracing of ROS 2,"](https://arxiv.org/abs/2201.00393) *IEEE Robotics and Automation Letters*, vol. 7, no. 3, pp. 6511–6518, 2022.
    <details>
    <summary>BibTeX</summary>

    ```bibtex
    @article{bedard2022ros2tracing,
      title={ros2\_tracing: Multipurpose Low-Overhead Framework for Real-Time Tracing of ROS 2},
      author={B{\'e}dard, Christophe and L{\"u}tkebohle, Ingo and Dagenais, Michel},
      journal={IEEE Robotics and Automation Letters},
      year={2022},
      volume={7},
      number={3},
      pages={6511--6518},
      doi={10.1109/LRA.2022.3174346}
    }
    ```
    </details>

This other paper leverages `ros2_tracing` to analyze and visualize the flow of messages across distributed ROS 2 systems:
* C. Bédard, P.-Y. Lajoie, G. Beltrame, and M. Dagenais, ["Message Flow Analysis with Complex Causal Links for Distributed ROS 2 Systems,"](https://arxiv.org/abs/2204.10208) *Robotics and Autonomous Systems*, vol. 161, p. 104361, 2023.
    <details>
    <summary>BibTeX</summary>

    ```bibtex
    @article{bedard2023messageflow,
      title={Message flow analysis with complex causal links for distributed {ROS} 2 systems},
      author={B{\'e}dard, Christophe and Lajoie, Pierre-Yves and Beltrame, Giovanni and Dagenais, Michel},
      journal={Robotics and Autonomous Systems},
      year={2023},
      volume={161},
      pages={104361},
      doi={10.1016/j.robot.2022.104361}
    }
    ```
    </details>

Finally, check out the following presentations:

* ROSCon 2023: "Improving Your Application's Algorithms and Optimizing Performance Using Trace Data" ([video](https://vimeo.com/879001159), [slides](https://roscon.ros.org/2023/talks/Improving_Your_Applications_Algorithms_and_Optimizing_Performance_Using_Trace_Data.pdf))
* ROS World 2021: "Tracing ROS 2 with ros2_tracing" ([video](https://vimeo.com/652633418), [slides](https://github.com/ros2/ros2_tracing/blob/rolling/doc/2021-10-20_ROS_World_2021_-_Tracing_ROS_2_with_ros2_tracing.pdf))

## Tutorials & demos

* ROS 2 documentation:
    * [Building ROS 2 with tracing](https://docs.ros.org/en/rolling/How-To-Guides/Building-ROS-2-with-Tracing.html)
    * [How to use `ros2_tracing` to trace and analyze an application](https://docs.ros.org/en/rolling/Tutorials/Advanced/ROS2-Tracing-Trace-and-Analyze.html)
* ROS World 2021 demo: [github.com/christophebedard/ros-world-2021-demo](https://github.com/christophebedard/ros-world-2021-demo)

## Building

As of Iron, the LTTng tracer is a ROS 2 dependency.
Therefore, ROS 2 can be traced out-of-the-box on Linux; this package does not need to be re-built.

To make sure that the instrumentation and tracepoints are available:

```
$ source /opt/ros/rolling/setup.bash  # With a binary install
$ source ./install/setup.bash  # When building from source
$ ros2 run tracetools status
Tracing enabled
```

A ROS 2 installation only includes the LTTng userspace tracer (LTTng-UST), which is all that is needed to trace ROS 2.
To trace the Linux kernel, the [LTTng kernel tracer](https://lttng.org/docs/v2.13/#doc-tracing-the-linux-kernel) must be installed separately:

```
$ sudo apt-get update
$ sudo apt-get install lttng-modules-dkms
```

For more information about LTTng, [refer to its documentation](https://lttng.org/docs/v2.13/).

### Removing the instrumentation

To build and remove all instrumentation, use `TRACETOOLS_DISABLED`:

```
$ colcon build --cmake-args -DTRACETOOLS_DISABLED=ON
```

This will remove all instrumentation from the core ROS 2 packages, and thus they will not depend on or link against the shared library provided by the [`tracetools` package](#tracetools).
This also means that LTTng is not required at build-time or at runtime.

### Excluding tracepoints

Alternatively, to only exclude the actual tracepoints, use `TRACETOOLS_TRACEPOINTS_EXCLUDED`:

```
$ colcon build --packages-select tracetools --cmake-clean-cache --cmake-args -DTRACETOOLS_TRACEPOINTS_EXCLUDED=ON
```

This will keep the instrumentation but remove all tracepoints.
This also means that LTTng is not required at build-time or at runtime.
This option can be useful, since tracepoints can be added back in or removed by simply replacing/re-building the shared library provided by the [`tracetools` package](#tracetools).

## Tracing

By default, trace data will not be generated, and thus these packages will have virtually no impact on execution.
LTTng has to be configured for tracing.
The packages in this repo provide two options: a [command](#trace-command) and a [launch file action](#launch-file-trace-action).

**Note**: tracing must be started before the application is launched.
Metadata is recorded during the initialization phase of the application.
This metadata is needed to understand the rest of the trace data, so if tracing is started after the application started executing, then the trace data might be unusable.
For more information, refer to the [design document](./doc/design_ros_2.md#general-guidelines).
The [launch file action](#launch-file-trace-action) is designed to automatically start tracing before the application launches.

The tracing directory can be configured using command/launch action parameters, or through environment variables with the following logic:

* Use `$ROS_TRACE_DIR` if `ROS_TRACE_DIR` is set and not empty.
* Otherwise, use `$ROS_HOME/tracing`, using `~/.ros` for `ROS_HOME` if not set or if empty.

Additionally, **if you're using kernel tracing with a non-root user, make sure that the [`tracing` group exists and that your user is added to it](https://lttng.org/docs/v2.13/#doc-tracing-group)**.

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

By default, it will enable all ROS 2 tracepoints.
The trace will be written to `~/.ros/tracing/session-YYYYMMDDHHMMSS`.
Run the command with `-h` for more information.

The `ros2 trace` command requires user interaction to start and then stop tracing.
To trace without user interaction (e.g., in scripts), or for finer-grained tracing control, the following sub-commands can be used:

```
$ ros2 trace start session_name   # Configure tracing session and start tracing
$ ros2 trace pause session_name   # Pause tracing after starting
$ ros2 trace resume session_name  # Resume tracing after pausing
$ ros2 trace stop session_name    # Stop tracing after starting or resuming
```

Run each command with `-h` for more information.

You must [install the kernel tracer](#building) if you want to enable kernel events (using the `-k`/`--kernel-events` option).
If you have installed the kernel tracer, use kernel tracing, and still encounter an error here, make sure to [add your user to the `tracing` group](#tracing).

### Launch file trace action

Another option is to use the `Trace` action in a Python, XML, or YAML launch file along with your `Node` action(s).
This way, tracing automatically starts when launching the launch file and ends when it exits or when terminated.

```
$ ros2 launch tracetools_launch example.launch.py
```

The `Trace` action will also set the `LD_PRELOAD` environment to preload [LTTng's userspace tracing helper(s)](https://lttng.org/docs/v2.13/#doc-prebuilt-ust-helpers) if the corresponding event(s) are enabled.
For more information, see [this example launch file](./tracetools_launch/launch/example.launch.py) and the [`Trace` action](./tracetools_launch/tracetools_launch/action.py).

You must [install the kernel tracer](#building) if you want to enable kernel events (`events_kernel` in Python, `events-kernel` in XML or YAML).
If you have installed the kernel tracer, use kernel tracing, and still encounter an error here, make sure to [add your user to the `tracing` group](#tracing).

## Design

See the [design document](./doc/design_ros_2.md).

## Real-time

LTTng-UST, the current default userspace tracer used for [tracing ROS 2](#overview), was designed for real-time production applications.
It is a low-overhead tracer with many important real-time compatible features:

* userspace tracer completely implemented in userspace, independent from the kernel
* reentrant, thread-safe, signal-safe, non-blocking
* no system calls in the fast path
* no copies of the trace data

However, some settings need to be tuned for it to be fully real-time safe and for performance to be optimal for your use-case:

* timers[^rt-1]: use [read timer](https://lttng.org/docs/v2.13/#doc-channel-read-timer) to avoid a write(2) call
* sub-buffer[^rt-1] count and size:
    * see [documentation](https://lttng.org/docs/v2.13/#doc-channel-subbuf-size-vs-subbuf-count) for sub-buffer count and size tuning tips based on your use-case
    * minimize sub-buffer count to minimize sub-buffer switching overhead
* one-time memory allocation/lock/syscall per thread:
    * usually done the first time a tracepoint is executed within a thread for URCU thread registration, but registration can be manually performed to force it to be done during your application's initialization
    * see [this LTTng mailing list message](https://lists.lttng.org/pipermail/lttng-dev/2019-November/029409.html)

[^rt-1]: this setting cannot currently be set through the [`Trace` launch file action](#launch-file-trace-action) or the [`ros2 trace` command](#trace-command), see [#20](https://github.com/ros2/ros2_tracing/issues/20)

For further reading:

* [LTTng documentation](https://lttng.org/docs/v2.13/)
* [Combined Tracing of the Kernel and Applications with LTTng](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.641.1965&rep=rep1&type=pdf#page=87): LTTng-UST architecture and design goals (section 3)
* [Survey and Analysis of Kernel and Userspace Tracers on Linux: Design, Implementation, and Overhead](https://dl.acm.org/doi/abs/10.1145/3158644): LTTng-UST overhead and design compared to other kernel and userspace tracers (table 6: average latency overhead per tracepoint of 158 ns)

The LTTng kernel tracer has a similar implementation, but is separate from the userspace tracer.

## Packages

### lttngpy

Package containing `liblttng-ctl` Python bindings.

### ros2trace

Package containing a `ros2cli` extension to enable tracing.

### tracetools

Library to support instrumenting ROS packages, including core packages.

This package claims to be in the **Quality Level 1** category, see the [Quality Declaration](./tracetools/QUALITY_DECLARATION.md) for more details.

See the [API documentation](https://docs.ros.org/en/rolling/p/tracetools/).

### tracetools_launch

Package containing tools to enable tracing through launch files.

### tracetools_read

Package containing tools to read traces.

### tracetools_test

Package containing tools for tracing-related tests.

### tracetools_trace

Package containing tools to enable tracing.

### test_ros2trace

Package containing system tests for `ros2trace`.

### test_tracetools

Package containing unit and system tests for `tracetools`.

### test_tracetools_launch

Package containing system tests for `tracetools_launch`.

## Analysis

See [`tracetools_analysis`](https://github.com/ros-tracing/tracetools_analysis).
