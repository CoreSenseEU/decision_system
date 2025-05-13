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

from rclpy.node import Node

from decision_msgs.msg import Choice, Decision


class AcceptNode(Node):
    """An abstract class to accept or reject a choice.
    """
    def __init__(self, policy):
        self.policy_ = f'accept_{policy}'
        self.policy_str = self.policy_ # mutable by children

        super().__init__(f'{self.policy_}_node')
        self.get_logger().info(f'Starting ACCEPT node with policy: {self.policy_}')

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
        try:
            # TODO: Validate msg.evaluation matrix?
            decision.accepted = self.accept(msg)
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        if decision.accepted:
            verb = 'Accepting'
        else:
            verb = 'Rejecting'
        self.get_logger().info(f'{verb} choice {decision.choice} with policy: {self.policy_str}')
        self.pub_.publish(decision)

    # To be overridden by children
    def accept(self, msg):
        raise NotImplementedError
