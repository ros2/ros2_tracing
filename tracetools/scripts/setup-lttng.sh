#!/bin/bash
# Copyright 2019 Robert Bosch GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# set up ust ros2 events
for event in \
  ros2:rcl_init \
  ros2:rcl_node_init \
  ros2:rcl_publisher_init \
  ros2:rcl_subscription_init \
  ros2:rclcpp_subscription_callback_added \
  ros2:rcl_service_init \
  ros2:rclcpp_service_callback_added \
  ros2:rcl_client_init \
  ros2:rcl_timer_init \
  ros2:rclcpp_timer_callback_added \
  ros2:rclcpp_register_callback \
  ros2:callback_start \
  ros2:callback_end
do
	lttng enable-event -c ros2 -u $event
done

# process context
lttng add-context -c ros2 -u \
  -t vpid -t procname \
  -t vtid -t perf:thread:instructions \
  -t perf:thread:cycles -t perf:thread:cpu-cycles

#kernel
lttng enable-channel --kernel kchan --subbuf-size=8M

# # network
# for event in net_dev_queue netif_receive_skb net_if_receive_skb
# do
#     lttng enable-event --kernel --channel=kchan $event
# done

# other kernel stuff
for event in sched_switch sched_waking sched_pi_setprio sched_process_fork sched_process_exit sched_process_free sched_wakeup\
    irq_softirq_entry irq_softirq_raise irq_softirq_exit irq_handler_entry irq_handler_exit\
    lttng_statedump_process_state lttng_statedump_start lttng_statedump_end lttng_statedump_network_interface lttng_statedump_block_device\
    block_rq_complete block_rq_insert block_rq_issue\
    block_bio_frontmerge sched_migrate sched_migrate_task power_cpu_frequency\
    net_dev_queue netif_receive_skb net_if_receive_skb\
    timer_hrtimer_start timer_hrtimer_cancel timer_hrtimer_expire_entry timer_hrtimer_expire_exit
do
    lttng enable-event --kernel --channel=kchan $event
done

# lttng enable-event -k --syscall --all
