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
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, ActionClient

from decision_msgs.msg import Evaluation, AssessmentMatrix
from decision_msgs.action import Aggregate
from abstract_decision_components.util import scores_to_np_array, load_cs_description


class AggregateMultiValueUtilityActionServer(Node):
    """Compute multi-valued utility scores for each alternative.

    :param children: Namespaces of each of the child aggregators that
        this multi-value utility agregator combines. Each of these children
        must publish its `evaluations` topic under its own namespace.
    """
    # TODO: if we switch from topics to services, then this behavior looks very
    #    similar to AssessNode and doesn't require buffering

    def __init__(self):
        self.policy_ = 'aggregate_multi_value_utility'
        super().__init__('aggregate_multi_value_utility_action_server')
        self.get_logger().info('Starting AGGREGATE action_server with policy: aggregate_multi_value_utility')

        self.declare_parameter('coresense_engine', load_cs_description(self.policy_))

        self.clients_ = {}

        self.add_on_set_parameters_callback(self._update_children)
        self.declare_parameter('children', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('timeout', 5.0)

        self.action_server_ = ActionServer(
                self,
                Aggregate,
                'Aggregate',
                self.aggregate_cb)

        self.pub_ = self.create_publisher(
                Evaluation,
                'evaluation',
                10)

    def aggregate_cb(self, goal_handle):
        assessments = goal_handle.request.assessments
        self.get_logger().info(f'Aggregating assessments of {assessments.cues} on {assessments.alternatives} with policy: aggregate_multi_value_utility')
        timeout = self.get_parameter('timeout').value
        then = self.get_clock().now().nanoseconds

        # start each client
        futures = []
        for child, client in self.clients_.items():
            client_gh_future = client.send_goal_async(Aggregate.Goal(assessments=assessments))
            timeout_sec = timeout - (self.get_clock().now().nanoseconds - then) / 10**9
            self.executor.spin_until_future_complete(client_gh_future, timeout_sec)
            if not client_gh_future.done():
                print(timeout - (self.get_clock().now().nanoseconds - then) / 10**9)
                self.get_logger().error(f'Failed to send goal to aggregator {child}: timed out')
                return self._cancel_all(futures)
            if client_gh_future.exception():
                self.get_logger().error(f'Failed to send goal to aggregator {child}: {str(client_gh_future.exception())}')
                return self._cancel_all(futures)

            client_gh = client_gh_future.result()
            futures.append((child, client_gh, client_gh.get_result_async()))

        # spin until all success or one failure
        results = {}
        i = 0
        while len(futures) > 0:
            timeout_sec = timeout - (self.get_clock().now().nanoseconds - then) / 10**9
            if timeout_sec < 0:
                self.get_logger().error(f'Aggregation timed out after {timeout_sec:.3f} seconds')
                return self._cancel_all(futures, goal_handle)
        
            if futures[i][2].cancelled():
                self.get_logger().error(f'Aggregator {futures[i][0]} was cancelled')
                return self._cancel_all(futures, goal_handle)

            if futures[i][2].done():
                if futures[i][2].exception():
                    self.get_logger().error(f'Aggregator {futures[i][0]} failed with exception: {str(futures[i][2].exception())}')
                    return self._cancel_all(futures, goal_handle)
                child, _, future = futures.pop(i)
                results.update({child: future.result().result}) # future result is a wrapped result
                goal_handle.publish_feedback(Aggregate.Feedback(num_judged=len(results)))
            else:
                self.executor.spin_once()
                i = (i + 1) % len(futures)

        # assemble result
        axes = []
        scores = []
        for child in self.children_:
            result = results[child]
            scores.append(scores_to_np_array(result.scores, len(assessments.alternatives)))
            axes += result.axes
        
        evaluation = Evaluation(alternatives=assessments.alternatives,
                                axes=axes,
                                scores=np.concatenate(scores, axis=1).flatten().tolist())

        self.pub_.publish(evaluation)

        goal_handle.succeed()
        return Aggregate.Result(axes=evaluation.axes,
                                scores=evaluation.scores)

    def _cancel_all(self, futures, goal_handle):
        for child, client_ch, future in futures:
            self.get_logger().error(f'Cancelling aggregator {child}')
            future.cancel()
            client_ch.cancel_goal_ansync()

        goal_handle.abort()
        return Aggregate.Result()

    def _update_children(self, parameters):
        # TODO: handle case when this is in the middle of fulfilling an action

        children = None
        for p in parameters:
            if p.name == 'children':
                children = p.value
                break
        if children is None:
            return SetParametersResult(successful=True)
        self.get_logger().info(f'Updating action clients for children: {children}')

        # TODO: use an LRU cache instead of updating these each time?
        # keep old clients matching children in the new parameters
        clients = {child: client for child, client in self.clients_.items() if child in children}

        # destroy old non-matching clients
        for child, client in self.clients_.items():
            if child not in children:
                client.destroy()

        # create a new client for each new cue
        clients.update({child: ActionClient(self,
                                            Aggregate,
                                            f'{child}/Aggregate')
                        for child in children if child not in self.clients_.keys()})
        self.children_ = children
        self.clients_ = clients
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)

    node = AggregateMultiValueUtilityActionServer()

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
