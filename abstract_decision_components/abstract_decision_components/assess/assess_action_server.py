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
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from decision_msgs.msg import AssessmentMatrix
from decision_msgs.srv import AssessAlternatives
from decision_msgs.action import Assess


class AssessActionServer(Node):
    """
    Assesses alternatives based on cues.

    :param cues: Namespaces of each of the cues that this assesses. Each of
        these cues are assumed to be unique ROS services.
    :param timeout: The total time (in seconds) to wait to assess all cues.
    """

    def __init__(self):
        super().__init__('assess_action_server')
        self.get_logger().info('Starting ASSESS action server')

        self.cues_ = {}
        self.alternatives = {}
        self.clients_ = {}
        self.cue_cb_group_ = ReentrantCallbackGroup()

        self.declare_parameter('timeout', 5.0)

        self.action_server_ = ActionServer(
                self,
                Assess,
                '~/Assess',
                self.assess_cb)

        self.pub_ = self.create_publisher(
                AssessmentMatrix,
                'assessments',
                10)

    def assess_cb(self, goal_handle):
        self._update_cues(goal_handle.request.cues)
        self.alternatives_ = goal_handle.request.alternatives 
        
        if len(self.alternatives_) < 1:
            self.get_logger().error('Recieved empty list of alternatives')
            goal_handle.abort()
            return Assess.Result()
        self.get_logger().info(f'Assessing alternatives {self.alternatives_} with cues {self.cues_}')

        # start each cue
        futures = []
        for cue, client in self.clients_.items():
            request = AssessAlternatives.Request(alternatives=self.alternatives_)
            futures.append((cue, client, client.call_async(request)))

        # TODO: switch to a spin-and-check model to fail eagerly
        # TODO: abstract this behavior because ASSESS_MULTIVALUE uses the same logic
        # wait for each cue to finish
        timeout = self.get_parameter('timeout').value
        scores = np.zeros((len(self.alternatives_), len(self.cues_)))
        success = True
        then = self.get_clock().now().nanoseconds
        for i, (cue, client, future) in enumerate(futures):
            # TODO: fail if the action was cancelled...
            if not success:
                client.remove_pending_request(future)
                continue

            timeout_sec = timeout - (self.get_clock().now().nanoseconds - then) / 10**9
            if timeout_sec > 0:
                self.executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
            if future.done():
                if future.exception() is not None:
                    self.get_logger().error(f'Cue {cue} failed with exception: {str(future.exception())}')
                    success = False
                else:
                    result = future.result()
                    if len(result.preferences) == 0:
                        self.get_logger().error(f'Cue {cue} failed to assess alternatives')
                        success = False
                    else:
                        scores[:,i] = [p.score for p in result.preferences]
                        goal_handle.publish_feedback(Assess.Feedback(num_assessed=i+1))
            else:
                client.remove_pending_request(future)
                self.get_logger().error(f'Cue {cue} timed out')
                success = False

        if not success:
            #  Kirsch says that all cues must be processed.
            self.get_logger().error("Failed to assess all cues")
            goal_handle.abort()
            return Assess.Result()

        # Also publish on the assessments topic
        # TODO: how much info should be published? Should this be a new "PartialDecision" message or something?
        assessments = AssessmentMatrix(cues=self.cues_, 
                                       alternatives=self.alternatives_,
                                       scores=scores.flatten().tolist())
        self.pub_.publish(assessments)

        goal_handle.succeed()
        self.get_logger().info(f'Succeeded with assessment matrix {assessments}')
        return Assess.Result(scores=assessments.scores)

    def _update_cues(self, cues):
        self.get_logger().info(f'Updating cues: {cues}')
        cue_ids = [cue.id for cue in cues]

        # TODO: use an LRU cache instead of updating these each time?
        # TODO: abstract this behavior because ASSESS_MULTIVALUE uses the same logic
        # keep old clients matching cues in the new goal
        clients = {cue: client for cue, client in self.clients_.items() if cue in cue_ids}

        # destroy old non-matching clients
        for cue, client in self.clients_.items():
            if cue not in cue_ids:
                while not self.destroy_client(client):
                    self.executor.spin_once()

        # create a new client for each new cue
        clients.update({cue: self.create_client(AssessAlternatives, cue,
                                                callback_group=self.cue_cb_group_)
                        for cue in cue_ids if cue not in self.clients_.keys()})
        self.cues_ = cues
        self.clients_ = clients


def main(args=None):
    rclpy.init(args=args)

    node = AssessActionServer()
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
