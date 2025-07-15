%%% Decision making problem state
% TODO: gap has been overloaded in the current decision logic as a generic 
%   data container. It should probably be renamed.

% NOTE: 0-airity type predicates are defined to fail to accomodate for potential
%   errors in Prolog Query parsing of empty strings.

%% Types

% A difference between current and desired understanding states in which a
% specific decision problem is to be solved to close the gap.
% 
% A unique gap is created whenever a decision is made.
%
% gap(?Gap)
:- dynamic gap/1.
:- multifile gap/1.
gap() :- fail.



%% Predicates

% An XML file implementing a behavior tree of a heuristic used to make a decision.
%
% Assume file path is relative to the `working_directory` parameter of the
% running `assemble_decision_heuristic` ROS node.
%
% heuristic_of(?XMLfile, ?Gap)
:- dynamic heuristic_of/2.
:- multifile heuristic_of/2.

% The ID of the behavior tree root node of a heuristic used to make a decision.
% TODO: change this to be relative to the heuristic instead of the GAP?
%
% entry_point_of(?BehaviorTreeID, ?Gap)
:- dynamic entry_point_of/2.
:- multifile entry_point_of/2.

% A YAML file describing a configuration of ROS parameters corresponding to a particular heuristic.
%
% Assume file path is relative to the `working_directory` parameter of the
% running `assemble_decision_heuristic` ROS node.
%
% config_of(?YAMLfile, ?Gap)
:- dynamic config_of/2.
:- multifile config_of/2.

% A condition to be implemented for each specific gap on the class of possible
% alternatives which would close the gap.
%
% alternative_of(?Alternative, ?Gap)
:- dynamic alternative_of/2.
:- multifile alternative_of/2.

% A condition to be implemented for each specific gap on the set of relevant
% cues which could be used to evaluate the alternatives of the gap (in the current iteration).
% Assume every cue is some ROS service.
%
% TODO: rename this to something indicating relevance instead. Maybe relevant_for/2 ?
%
% cue_of(?RosService, ?Gap)
:- dynamic cue_of/2.
:- multifile cue_of/2.

% A condition to be implemented for each specific gap on the set of possible
% cues which could be used to evaluate the alternatives of the gap.
% Assume every cue is some ROS service.
%
% available_for(?RosService, ?Gap)
:- dynamic available_for/2.
:- multifile available_for/2.

% When a decision is made, the gap is closed with all members of the chosen set of alternatives.
%
% closed_with(?Gap, ?Alternative)
:- dynamic closed_with/2.

% When an alternative is matched with the `alternative_of/2` predicate, it should
% be marked as `fetched_for` a particular gap.
%
% fetched_for(?Alternative, ?Gap)
:- dynamic fetched_for/2.



%% Functions

% Copy an existing gap, creating a new one with a uuid but using the same
% heuristic, config, alternative class, and cues.
% Notably not copied are the `closed_with` and `fetched_for` predicates.
%
% duplicate_gap(--NewGap, ++OldGap)
duplicate_gap(Gnew, Gold) :-
    gap(Gold), 
    uuid(ID), sub_atom(ID, 0, 8, _, Suffix), atom_concat(gap_, Suffix, Gnew),
    assertz((gap(Gnew))), 
    assertz((alternative_of(A, Gnew) :- alternative_of(A, Gold))),
    assertz((cue_of(C, Gnew) :- cue_of(C, Gold))),
    assertz((available_for(C, Gnew) :- available_for(C, Gold))),
    assertz((heuristic_of(Xml_file, Gnew) :- heuristic_of(Xml_file, Gold))),
    assertz((entry_point_of(BT, Gnew) :- entry_point_of(BT, Gold))),
    assertz((config_of(Yaml_file, Gnew) :- config_of(Yaml_file, Gold))).

% Create a new gap for deciding cues based on a gap for deciding alternatives
%
% create_cue_gap_from(--CueGap, ++AlternativeGap)
create_cue_gap_from(Gnew, Gold) :-
    gap(Gold), 
    uuid(ID), sub_atom(ID, 0, 8, _, Suffix), atom_concat(cues_gap_, Suffix, Gnew),
    assertz((gap(Gnew))), 
    assertz((alternative_of(A, Gnew) :- available_for(A, Gold), \+ fetched_for(A, Gold))),
    % assertz((alternative_of(A, Gnew) :- cue_of(A, Gold))),

    % For now, always use Take-The-Best
    assertz((cue_of(A, Gold) :- closed_with(Gnew, A))), % This prevents cues that were previously tried from being used again.
    assertz((cue_of('/validity', Gnew))),
    assertz((heuristic_of(Path, Gnew) :- 
              ament_package_share_prefix(Dir), 
              atom_concat(Dir, 'behavior_trees/decision_heuristics/take_the_best.xml', Path))),
    assertz((entry_point_of('DecideOnGap_cues_ttb_gap', Gnew))),
    assertz((config_of(Path, Gnew) :- 
              ament_package_share_prefix(Dir), 
              atom_concat(Dir, 'config/decision_heuristics/take_the_best.yaml', Path))).


% The directory of the `krr_btcpp_ros2` package, as an atom.
% TODO: get this from ROS instead
%
% ament_package_share_prefix(--Directory)
ament_package_share_prefix('/mnt/Shared/School/Thesis/krr_ws/install/krr_btcpp_ros2/share/krr_btcpp_ros2/').




