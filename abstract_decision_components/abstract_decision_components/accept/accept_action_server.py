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
from rclpy.action import ActionServer

from decision_msgs.msg import Decision
from decision_msgs.action import Accept


class AcceptActionServer(Node):
    """An abstract class to accept or reject a choice.
    """
    def __init__(self, policy):
        self.policy_ = f'accept_{policy}'
        self.policy_str = self.policy_ # mutable by children

        super().__init__(f'{self.policy_}_action_server')
        self.get_logger().info(f'Starting ACCEPT action server with policy: {self.policy_}')

        self.action_server_ = ActionServer(
                self,
                Accept,
                '~/Accept',
                self.accept_cb)
        
        self.pub_ = self.create_publisher(
                Decision,
                'decision',
                10)

    def accept_cb(self, goal_handle):
        choice = goal_handle.request.choice
        decision = Decision(choice=choice.chosen)
        self.get_logger().info(f'Accepting choice {decision.choice} with policy: {self.policy_str}')

        try:
            # TODO: Validate msg.evaluation matrix?
            decision.accepted = self.accept(choice)
        except ValueError as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            return Accept.Result()

        if decision.accepted:
            verb = 'accepted'
        else:
            verb = 'rejected'
        self.get_logger().info(f'Successfully {verb} choice {decision.choice} with policy: {self.policy_str}')
        self.pub_.publish(decision)

        goal_handle.succeed()
        return Accept.Result(accepted=decision.accepted)

    # To be overridden by children
    def accept(self, msg):
        raise NotImplementedError
