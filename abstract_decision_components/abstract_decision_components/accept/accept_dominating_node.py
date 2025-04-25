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

from decision_msgs.msg import Choice, Decision
from abstract_decision_components.accept import accept


class AcceptDominatingNode(Node):
    """
    Accepts a choice if the scores of all chosen alternatives are strictly
    greater than the best unchosen alternative for each requested axis.

    :param axes: A list of strings of axes to check for dominance.
    """
    def __init__(self):
        super().__init__('accept_dominating_node')
        self.get_logger().info('Starting ACCEPT node with policy: accept_dominating')
        
        self.declare_parameter('axes', Parameter.Type.STRING_ARRAY)

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
        try:
            decision.success, decision.reason = accept.dominating(msg.chosen, msg.evaluation, axes=axes)
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        if decision.success:
            verb = 'Accepting'
        else:
            verb = 'Rejecting'
        self.get_logger().info(f'{verb} choice {decision.choice} with policy: accept_dominating, "{axes}"')
        self.pub_.publish(decision)
 

def main(args=None):
    rclpy.init(args=args)

    node = AcceptDominatingNode()

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
