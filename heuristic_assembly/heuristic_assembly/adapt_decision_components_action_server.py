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
import os
import yaml

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.action import ActionServer
# from ros2param.api import call_get_parameters, call_set_parameters

# TODO: backport this from ROS 2 rolling?
# from rclpy.parameter import parameter_dict_from_yaml_file
from heuristic_assembly.rclpy_parameter_rolling import parameter_dict_from_yaml_file

from rcl_interfaces.srv import SetParameters, GetParameters
from decision_msgs.action import AdaptDecisionComponents


class AdaptDecisionComponentsActionServer(Node):
    """
    An action to adapt decision components with new parameters.

    :param heuristic_directory: The working directory where new heuristics and
        parameter files are saved. Defaults to the current working directory.

    :param bt_executor: The name of the node to execute behavior trees. Defaults to '/bt_executor'.

    """
    def __init__(self):
        super().__init__('adapt_decision_components_action_server')
        self.get_logger().info('Starting ADAPT_DECISION_COMPONENTS action server')

        self.declare_parameter('bt_executor', '/bt_executor')
        self.declare_parameter('working_directory', os.getcwd())

        self.action_server_ = ActionServer(
                self,
                AdaptDecisionComponents,
                'AdaptDecisionComponents',
                self.adapt_cb)

    def adapt_cb(self, goal_handle):
        params_file = goal_handle.request.params_file
        with open(params_file, 'r') as f:
            nodes = list(yaml.safe_load(f).keys())

        bt_executor = self.get_parameter('bt_executor').value

        if len(nodes) < 1:
            reason = 'Recieved empty list of parameters'
            self.get_logger().error(reason)
            goal_handle.abort()
            return AdaptDecisionComponents.Result(success=False, reason=reason)
        self.get_logger().info(f'Adapting {nodes}')

        # TODO: just generate the parameters by hand and remove this dependency
        parameters = [list(parameter_dict_from_yaml_file(params_file, target_nodes=[node]).values())
                      for node in nodes]

        # Add bt_execuor so that the behavior tree can be added
        # TODO: include a prolog entry for `engine(bt_executor), has_ros_node(???, bt_executor)`
        heuristic_dir = os.path.join(self.get_parameter('working_directory').value, 'heuristics')
        client = self.create_client(GetParameters, bt_executor + "/get_parameters")
        request = GetParameters.Request(names=['behavior_trees'])
        response = self.call_service(client, request)
        if response is None or len(response.values) != 1:
            reason = f"Failed to get 'behavior_trees' parameter for {bt_executor}"
            self.get_logger().error(reason)
            goal_handle.abort()
            return AdaptDecisionComponents.Result(success=False, reason=reason)

        nodes.append(bt_executor)
        parameters.append([Parameter('behavior_trees', response.values[0].string_array_value.append(heuristic_dir))])

        # TODO: switch to a parallel model instead of series?
        # Right now assume that the services return quickly so it's not too much of a difference
        success = True
        for i, (node, params) in enumerate(zip(nodes, parameters)):
            if success:
                success, reason = self.adapt_decision_component(node, params)
                goal_handle.publish_feedback(AdaptDecisionComponents.Feedback(num_adapted=i, num_to_adapt=len(parameters)))
            else:
                # TODO: should successful parameters be set back to how they started?
                goal_handle.abort()
                return AdaptDecisionComponents.Result(success=False, reason=reason)

        goal_handle.succeed()
        self.get_logger().info('Successfully adapted engines')
        return AdaptDecisionComponents.Result(success=success)

    def adapt_decision_component(self, node_name, parameters):
        # TODO: maybe use this instead of hand-rolled clients?
        #    See: https://github.com/ros2/ros2cli/blob/e77104637de7b4b8f9fa0f02210554c789e465b6/ros2param/ros2param/api/__init__.py#L35
        # client = AsyncParameterClient(self, node_name)

        client = self.create_client(SetParameters, node_name + "/set_parameters")
        request = SetParameters.Request(parameters=parameters)
        response = self.call_service(client, request)

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

    def _get_executor_params(self):
        # TODO: include a prolog entry for `engine(bt_executor), has_ros_node(???, bt_executor)`
        bt_executor = self.get_parameter('bt_executor').value
        heuristic_dir = os.path.join(self.get_parameter('working_directory').value, 'heuristics')

        client = self.create_client(GetParameters, bt_executor + "/get_parameters")
        request = GetParameters.Request(names=['behavior_trees'])
        response = self.call_service(client, request)
        if response is None or len(response.values) != 1:
            raise RuntimeError(f"Failed to get 'behavior_trees' parameter for {bt_executor}")
        
        params = [Parameter('behavior_trees', response.values[0].string_array_value.append(heuristic_dir))]
        return bt_executor, params 

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
