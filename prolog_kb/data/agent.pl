%%% The agent model of it's own capabilities and running processes

%% Types

% An engine running on the agent
%
% engine(?Engine)
:- dynamic engine/1.
:- discontiguous engine/1.



%% Predicates

% The YAML file describing parameters a particular engine is (intended to be) configured with.
%
% TODO: Right now ROS nodes with different parameter configurations are assumed
%    to be different engines. In the future these should be modeled independently.
%
% uses_ros_params(?Engine, ?YAMLfile)
:- dynamic uses_ros_params/2.
:- discontiguous uses_ros_params/2.

% The current name of a running ROS node implementing(?) a particular engine.
%
% has_ros_node(?Engine, ++RosNodeName)
:- dynamic has_ros_node/2.
:- discontiguous has_ros_node/2.

% An XML file implementing a behavior tree that executes a particular engine
% when it is ticked. This may include fallback behaviors, ensuring inputs exist,
% and preconditions are satisfied.
%
% Assume file path is relative to the `heuristic_assembly` ROS package
%
% has_meta(?Engine, ++YAMLfile)
:- dynamic has_meta/2.
:- discontiguous has_meta/2.
