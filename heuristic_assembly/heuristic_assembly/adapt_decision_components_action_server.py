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
from rclpy.parameter import parameter_dict_from_yaml_file

from rcl_interfaces.srv import SetParameters
from decision_msgs.action import AdaptDecisionComponents


class AdaptDecisionComponentsActionServer(Node):
    """
    An action to adapt decision components with new parameters.
    """
    def __init__(self):
        super().__init__('adapt_decision_components_action_server')
        self.get_logger().info('Starting ADAPT_DECISION_COMPONENTS action server')

        self.action_server_ = ActionServer(
                self,
                AdaptDecisionComponents,
                'AdaptDecisionComponents',
                self.adapt_cb)

    def adapt_cb(self, goal_handle):
        # TODO: instead of custom messages use this!!
        configurations = parameter_dict_from_yaml_file(goal_handle.request.file_path)

        if len(configurations) < 1:
            reason = 'Recieved empty list of configurations'
            self.get_logger().error(reason)
            goal_handle.abort()
            return AdaptDecisionComponents.Result(success=False, reason=reason)
        self.get_logger().info(f'Adapting {[c.node_name for c in configurations]}')

        # TODO: switch to a parallel model instead of series?
        # Right now assume that the services return quickly so it's not too much of a difference
        success = True
        for i, config in enumerate(configurations.items()):
            if success:
                success, reason = self.adapt_decision_component(config[0], config[1])
                goal_handle.publish_feedback(AdaptDecisionComponents.Feedback(num_configured=i))
            else:
                # TODO: should successful configurations be set back to how they started?
                goal_handle.abort()
                return AdaptDecisionComponents.Result(success=False, reason=reason)

        goal_handle.succeed()
        return AdaptDecisionComponents.Result(success=success, reason=reason)

    def adapt_decision_component(self, node_name, parameters):
        # TODO: maybe use this instead of hand-rolled clients?
        #    See: https://github.com/ros2/ros2cli/blob/e77104637de7b4b8f9fa0f02210554c789e465b6/ros2param/ros2param/api/__init__.py#L35
        # client = AsyncParameterClient(self, node_name)


        client = self.create_client(SetParameters, node_name + "/set_parameters")

        request = SetParameters.Request(parameters=parameters)
        response = self.call_service(client, request)

        while not self.destroy_client(client):
            self.executor.spin_once()

        if response is None:
            return False, f'Failed to set parameters for {node_name}'

        # Log all failed parameters
        success = True
        for i, result in enumerate(response.results):
            if not result.successful:
                self.get_logger().error(
                        f'Failed to set {request.parameters[i]}: {response.reason}')
                success = False

        if not success:
            return False, f'Failed to set all parameters for {node_name}'
        return True, ''

    def call_service(self, cli, request):
        """
        Adopted from https://github.com/kas-lab/krr_mirte_skills/blob/main/krr_mirte_skills/krr_mirte_skills/get_object_info.py
        """
        if cli.wait_for_service(timeout_sec=5.0) is False:
            self.get_logger().error(
                'service not available {}'.format(cli.srv_name))
            return None
        future = cli.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=5.0)
        if future.done() is False:
            self.get_logger().error(
                'Future not completed {}'.format(cli.srv_name))
            return None
        return future.result()


def main(args=None):
    rclpy.init(args=args)

    node = AdaptDecisionComponentsActionServer()

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
