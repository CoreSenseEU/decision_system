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
from rclpy.action import ActionServer

from decision_msgs.action import AssembleHeuristic


class AssembleDecisionHeuristicActionServer(Node):
    """
    An action to assemble a new decision heuristic.
    """
    def __init__(self):
        super().__init__('assemble_decision_heuristic_action_server')
        self.get_logger().info('Starting ASSEMBLE_DECISION_HEURISTIC action server')

        self.action_server_ = ActionServer(
                self,
                AssembleHeuristic,
                'AssembleHeuristic',
                self.assemble_cb)

    def assemble_cb(self, goal_handle):
        configurations = goal_handle.request.configurations

        if len(configurations) < 1:
            reason = 'Recieved empty list of configurations'
            self.get_logger().error(reason)
            goal_handle.abort()
            return AdaptDecisionComponents.Result(success=False, reason=reason)
        self.get_logger().info(f'Adapting {[c.node_name for c in configurations]}')

        # TODO: switch to a parallel model instead of series?
        # Right now assume that the services return quickly so it's not too much of a difference
        success = True
        for i, config in enumerate(configurations):
            if success:
                success, reason = self.adapt_decision_component(config.node_name, config.parameters)
                goal_handle.publish_feedback(AdaptDecisionComponents.Feedback(num_configured=i))
            else:
                # TODO: should successful configurations be set back to how they started?
                goal_handle.abort()
                return AdaptDecisionComponents.Result(success=False, reason=reason)

        goal_handle.succeed()
        return AdaptDecisionComponents.Result(success=success, reason=reason)

    def write_to_yaml(self):
        ...

    def write_to_xml(self):
        ...


def main(args=None):
    rclpy.init(args=args)

    node = AssembleDecisionHeuristicActionServer()

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
