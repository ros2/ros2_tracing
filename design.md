# Design

Plan for ROS 2 tracing & analysis effort.

## Steps

This general effort will be split into a few distinct steps.

### Instrumentation

The first goal is to statically instrument ROS 2, aiming for it to be in the ROS 2 E-turtle release (Nov 2019).

This includes transposing the existing ROS 1 instrumentation to ROS 2, wherever applicable. This step may not include instrumenting DDS implementations, and thus may be limited to the layer right before `rmw`.

The plan is to use LTTng with a ROS wrapper package like `tracetools` for ROS 1.

### Analysis & visualization

After the initial instrumentation, some general statistics analyses can be built. The targeted analysis & visualization tools are pandas and Jupyter. The goal is to make analyses general enough to be useful for different use-cases, e.g.:

* Callback duration
* Time between callbacks (between two callback starts and/or a callback end and a start)
* Message age (as the difference between processing time and message timestamp)
* Message size
* Memory usage
* Execution time/proportion accross a process' nodes/components
* Interruptions (noting that these may be more useful as time-based metrics instead of overall statistics):
    * scheduling events during a callback
    * delay between the moment a thread becomes ready and when it's actually scheduled
    * CPU cycles

with mean, stdev, etc. when applicable.

Generic tracepoints should also be provided for ROS 2 user code, which could then be applied to a user-provided model for higher-level behaviour statistics.


### Tools/accessibility

To make tracing ROS 2 more accessible and easier to adopt, we can put effort into integrating LTTng session setup & recording into the ROS 2 launch system.

This might include converting existing `tracetools` scripts to more flexible Python scripts, and then plugging that into the launch system.

### ROS 1/2 compatibility

Finally, we could look into making analyses work on both ROS 1 and ROS 2, through a common instrumentation interface (or other abstraction).
