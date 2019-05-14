# Design

Plan for ROS 2 tracing & analysis effort.

## Steps

This general effort will be split into a few distinct steps.

### Instrumentation

The first goal is to statically instrument ROS 2, aiming for it to be in the ROS 2 E-turtle release (Nov 2019).

This includes transposing the existing ROS 1 instrumentation to ROS 2, wherever applicable. This step may not include instrumenting DDS implementations, and thus may be limited to the layer right before `rmw`.

The plan is to use LTTng with a ROS wrapper package like `tracetools` for ROS 1.

### Analysis

After the initial instrumentation, some general statistics analyses can be built. The goal is to make those general enough to be useful for different use-cases, e.g.:

* Callback duration (mean, stdev, etc.)
* 
* 

Generic tracepoints should also be provided for ROS 2 user code, which could then be applied to a user-provided model for higher-level behaviour statistics.


### Tools/accessibility

To make tracing ROS 2 more accessible and easier to adopt, we can put effort into integrating LTTng session setup & recording into the ROS 2 launch system.

This might include converting existing `tracetools` scripts to more flexible Python scripts, and then plugging that into the launch system.

### ROS 1/2 compatibility

Finally, we could look into making the instrumentation work on both ROS 1 and ROS 2, through a common interface (or other abstraction).
