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

from decision_msgs.msg import Choice, Decision, Feature
import accept


class AcceptSatisficingNode(Node):
    """Accepts a choice if the scores of all chosen alternatives are
    greater than or equal to the threshold values of each requested axis.
    """
    def __init__(self):
        super().__init__('accept_satisficing_node')
        self.get_logger().info('Starting ACCEPT node with policy: accept_satisficing')

        self.declare_parameter('axies', [])
        self.declare_parameter('thresholds', [])

        self.sub_ = self.create_subscription(
                Choice,
                'choice',
                self.choice_cb,
                10)
        self.pub_ = self.create_publisher(
                Decision,
                'decision',
                10)

    def choice_cb(self, msg):
        raise NotImplementedError("Not yet been tested")
        decision = Decision(choice=msg.choice)
        axies = self.get_parameter('axies').get_parameter_value().string_array_value
        scores = self.get_parameter('thresholds').get_parameter_value().double_array_value
        features = [Feature(axis=a, score=s) for a, s in zip(axies, scores)]
        decision.reason, decision.success = accept.satisficing(msg.choice, msg.evaluation, features)

        if decision.success:
            verb = 'Accepting'
        else:
            verb = 'Rejecting'
        self.get_logger().info(f'{verb} choice {msg.choice} with policy: accept_satisficing, "{zip(axies,scores)}"')
        self.pub_.publish(decision)


def main(args=None):
    rclpy.init(args=args)

    node = AcceptSatisficingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
