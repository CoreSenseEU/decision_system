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
from rclpy.exceptions import ParameterException

from decision_msgs.msg import Choice, Decision, Feature
from abstract_decision_components.accept import accept


class AcceptSatisficingNode(Node):
    """Accepts a choice if the scores of all chosen alternatives are
    greater than or equal to the threshold values of each requested axis.

    :param axes: A list of strings of axes to check if satisficing. Must be the
        same length as ``thresholds``.
    :param thresholds: A list of doubles of thresholds for each axis. Must be
        the same length as ``axes``.
    """
    def __init__(self):
        super().__init__('accept_satisficing_node')
        self.get_logger().info('Starting ACCEPT node with policy: accept_satisficing')

        self.declare_parameter('axes', Parameter.Type.STRING_ARRAY) 
        self.declare_parameter('thresholds', Parameter.Type.DOUBLE_ARRAY)

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
        decision = Decision(choice=msg.chosen)
        axes = self.get_parameter('axes').value
        thresholds = self.get_parameter('thresholds').value
        self._check_params(axes, thresholds)

        features = list(zip(axes, thresholds))
        try:
            decision.success, decision.reason = accept.satisficing(msg.chosen, msg.evaluation, features)
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        if decision.success:
            verb = 'Accepting'
        else:
            verb = 'Rejecting'
        self.get_logger().info(f'{verb} choice {msg.chosen} with policy:' \
                             + f' accept_satisficing, "{features}"')
        self.pub_.publish(decision)

    def _check_params(self, axes, thresholds):
        if len(axes) != len(thresholds):
            raise ParameterException('Parameters have unequal lengths', ['axes', 'thresholds'])


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
