# Copyright 2025 KAS-Lab
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

import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from decision_msgs.msg import AssessmentArray, AlternativeArray, Assessment, Cue
from decision_msgs.srv import AssessAlternatives


class AssessNode(Node):
    """
    Assesses alternatives based on cues.

    :param cues: Namespaces of each of the cues that this assesses. Each of
        these cues are assumed to be unique ROS services.
    :param timeout: The total time (in seconds) to wait to assess all cues.
    """

    def __init__(self):
        super().__init__('assess_node')
        self.get_logger().info('Starting ASSESS node')

        self.cues_ = {}
        self.clients_ = {}
        self.cue_cb_group_ = ReentrantCallbackGroup()

        self.add_on_set_parameters_callback(self._update_cues)
        self.declare_parameter('cues', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('timeout', 5.0)

        self.sub_alternatives_ = self.create_subscription(
                AlternativeArray,
                'alternatives',
                self.alternatives_cb,
                10)
        
        self.pub_ = self.create_publisher(
                AssessmentArray,
                'assessments',
                10)

    def alternatives_cb(self, msg):
        if len(msg.alternatives) < 1:
            self.get_logger().error('Recieved empty list of alternatives')
            return
        self.get_logger().info(f'Assessing alternatives {msg.alternatives} with cues {self.cues_}')

        # start each cue
        futures = []
        for cue, client in self.clients_.items():
            request = AssessAlternatives.Request(alternatives=msg.alternatives)
            futures.append((cue, client, client.call_async(request)))

        # wait for each cue to finish
        timeout = self.get_parameter('timeout').value()
        assessments = []
        then = self.get_clock().now().nanoseconds
        for cue, client, future in futures:
            timeout_sec = timeout - (self.get_clock().now().nanoseconds - then) / 10**9
            if timeout_sec > 0:
                self.executor.spin_until_future_complete(future, timeout_sec=timeout_sec)

            if future.done():
                result = future.result()
                # TODO: do we even need ids anymore since seach service is unique?
                #   Keeping for now in case we want to switch cues to topics instead of services
                assessments.append(Assessment(cue=Cue(id=cue[1:], service=cue), preferences=result.preferences))
            else:
                client.remove_pending_request(future)
                self.get_logger().error(f'Cue {cue} failed or timed out')

        if len(assessments) != len(self.cues_):
            #  Kirsch says that all cues must be processed.
            self.get_logger().error("Failed to assess all cues")
            return

        self.pub_.publish(AssessmentArray(assessments=assessments))

    def _update_cues(self, parameters):
        cues = None
        for p in parameters:
            if p.name == 'cues':
                cues = p.value
                break
        if cues is None:
            return SetParametersResult(successful=True)

        # destroy old clients
        for client in self.clients_:
            while not self.destroy_client(client):
                self.executor.spin_once()

        # Create a new client for each cue
        self.get_logger().info(f'Updating cues: {cues}')
        self.cues_ = cues
        self.clients_ = []
        for cue in cues:
            client = self.create_client(
                    AssessAlternatives,
                    cue,
                    callback_group=self.cue_cb_group_)
            self.clients_.update({cue: client})

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)

    node = AssessNode()
    # TODO: figure out how to force the MTE to recieve the exception
    # executor = MultiThreadedExecutor()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
