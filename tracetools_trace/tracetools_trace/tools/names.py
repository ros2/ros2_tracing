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

"""Lists of names (events, context) for tracing."""

EVENTS_KERNEL = [
    'block_rq_complete',
    'block_rq_insert',
    'block_rq_issue',
    'block_bio_frontmerge',
    'irq_softirq_entry',
    'irq_softirq_raise',
    'irq_softirq_exit',
    'irq_handler_entry',
    'irq_handler_exit',
    'kmem_mm_page_alloc',
    'kmem_mm_page_free',
    'lttng_statedump_process_state',
    'lttng_statedump_start',
    'lttng_statedump_end',
    'lttng_statedump_network_interface',
    'lttng_statedump_block_device',
    'net_dev_queue',
    'netif_receive_skb',
    'net_if_receive_skb',
    'power_cpu_frequency',
    'sched_switch',
    'sched_waking',
    'sched_pi_setprio',
    'sched_process_fork',
    'sched_process_exit',
    'sched_process_free',
    'sched_wakeup',
    'sched_migrate',
    'sched_migrate_task',
    'timer_hrtimer_start',
    'timer_hrtimer_cancel',
    'timer_hrtimer_expire_entry',
    'timer_hrtimer_expire_exit',
]

# Kernel events that are currently used by analyses (or will most likely be used in the future)
DEFAULT_EVENTS_KERNEL = [
    'sched_switch',
    'kmem_mm_page_alloc',
    'kmem_mm_page_free',
    'power_cpu_frequency',
]

DEFAULT_EVENTS_ROS = [
    'ros2:rcl_init',
    'ros2:rcl_node_init',
    'ros2:rcl_publisher_init',
    'ros2:rcl_subscription_init',
    'ros2:rclcpp_subscription_init',
    'ros2:rclcpp_subscription_callback_added',
    'ros2:rcl_service_init',
    'ros2:rclcpp_service_callback_added',
    'ros2:rcl_client_init',
    'ros2:rcl_timer_init',
    'ros2:rclcpp_timer_callback_added',
    'ros2:rclcpp_callback_register',
    'ros2:callback_start',
    'ros2:callback_end',
]

DEFAULT_EVENTS_UST = DEFAULT_EVENTS_ROS

CONTEXT_TYPE_CONSTANTS_MAP = {
    'pid': 'EVENT_CONTEXT_PID',
    'procname': 'EVENT_CONTEXT_PROCNAME',
    'prio': 'EVENT_CONTEXT_PRIO',
    'nice': 'EVENT_CONTEXT_NICE',
    'vpid': 'EVENT_CONTEXT_VPID',
    'tid': 'EVENT_CONTEXT_TID',
    'vtid': 'EVENT_CONTEXT_VTID',
    'ppid': 'EVENT_CONTEXT_PPID',
    'vppid': 'EVENT_CONTEXT_VPPID',
    'pthread_id': 'EVENT_CONTEXT_PTHREAD_ID',
    'hostname': 'EVENT_CONTEXT_HOSTNAME',
    'ip': 'EVENT_CONTEXT_IP',
    'interruptible': 'EVENT_CONTEXT_INTERRUPTIBLE',
    'preemptible': 'EVENT_CONTEXT_PREEMPTIBLE',
    'need_reschedule': 'EVENT_CONTEXT_NEED_RESCHEDULE',
    'migratable': 'EVENT_CONTEXT_MIGRATABLE',
    'perf:thread:instructions': None,
    'perf:thread:cycles': None,
    'perf:thread:cpu-cycles': None,
}

CONTEXT = list(CONTEXT_TYPE_CONSTANTS_MAP.keys())

DEFAULT_CONTEXT = [
    'procname',
    'vpid',
    'vtid',
]
