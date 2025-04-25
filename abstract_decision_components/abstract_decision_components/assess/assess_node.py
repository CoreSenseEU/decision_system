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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from decision_msgs.msg import AssessmentArray, AlternativeArray, CueArray, Assessment
from decision_msgs.srv import AssessAlternatives


class AssessNode(Node):
    """
    Assesses alternatives based on cues.
    """

    def __init__(self, srv_timeout=5.0):
        """
        Assesses alternatives based on cues.

        :param srv_timeout: The total time (in seconds) to wait to assess all cues.
        """
        super().__init__('assess_node')
        self.get_logger().info('Starting ASSESS node')

        self.srv_timeout_ = srv_timeout # seconds

        self.sub_cues_ = self.create_subscription(
                CueArray,
                'cues',
                self.update_cues_cb,
                10)
        self.cues_ = {}
        self.cue_clients_ = {}
        self.cue_cb_group_ = ReentrantCallbackGroup()

        self.sub_alternatives_ = self.create_subscription(
                AlternativeArray,
                'alternatives',
                self.alternatives_cb,
                10)
        
        self.pub_ = self.create_publisher(
                AssessmentArray,
                'assessments',
                10)

    def alternatives_cb(self, msg):
        if len(msg.alternatives) < 1:
            self.get_logger().error('Recieved empty list of alternatives')
            return
        self.get_logger().info(f'Assessing alternatives {msg.alternatives} with cues {self.cues_}')

        # start each cue
        futures = []
        for cue_id, client in self.cue_clients_.entries():
            request = AssessAlternatives.Request(alternatives=msg.alternatives)
            futures.append((self.cues_[cue_id], client, client.call_async(request)))

        # wait for each cue to finish
        assessments = []
        then = self.get_clock().now().nanoseconds
        for cue, client, future in futures:
            timeout_sec = self._srv_timeout_ - (self.get_clock().now().nanoseconds - then) / 10**9
            if timeout_sec > 0:
                self.executor.spin_until_future_complete(future, timeout_sec=timeout_sec)

            if future.done():
                result = future.result()
                assessments.append(Assessment(cue=cue, preference=result.preferences))
            else:
                client.remove_pending_request(future)
                self.get_logger().error(f'Cue {cue} failed or timed out')

        if len(assessments) != len(self.cues_):
            #  Kirsch says that all cues must be processed.
            self.get_logger().error("Failed to assess all cues")
            return

        self.pub_.publish(AssessmentArray(assessments=assessments))

    def update_cues_cb(self, msg):
        for client in self.cue_clients_:
            self.destroy_client(client)

        self.cues_ = {cue.id : cue for cue in msg.cues}
        self.cues_clients_ = {}
        for cue in msg.cues:
            client = self.create_client(
                    AssessAlternatives,
                    cue.service,
                    callback_group=self.cue_cb_group_)
            self.cue_clients_.update({cue.id, client})

    # def assess(self, cue, alternatives):
    #     """Adapted from https://github.com/kas-lab/krr_mirte_skills/blob/main/krr_mirte_skills/krr_mirte_skills/get_object_info.py
    #     """
    #     request = AssessAlternatives.Request()
    #     request.alternatives = alternatives
    #
    #     # wait for service to be ready
    #     client = self.cue_clients_[cue.id]
    #     then = self.get_clock().now()
    #     if client.wait_for_service(timeout_sec=self.srv_timeout_) is False:
    #         self.get_logger().error(f'Service {cue.service} not available for cue {cue.id}')
    #         return None
    #
    #     # wait for service to complete
    #     future = client.call_async(request)
    #     time_waited = self.get_clock().now() - then
    #     self.executor.spin_until_future_complete(future, timeout_sec=(time_waited.nanoseconds / 10**9))
    #     if future.done() is False:
    #         self.get_logger().error(f'Service not completed or timed out for cue {cue.id}')
    #         return None
    #     result = future.result()
    #
    #     assessment = Assessment(cue=cue, preference=result.preferences)
    #     return assessment
    #
    # def start_assessment_async(self, client, alternatives):
    #     """Start an asyncronous assessment.
    #     """
    #     request = AssessAlternatives.Request()
    #     request.alternatives = alternatives
    #
    #     future = client.call_async(AssessAlternatives.Request(alternatives=alternatives))
    #     return future


def main(args=None):
    rclpy.init(args=args)

    node = AssessNode()
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
