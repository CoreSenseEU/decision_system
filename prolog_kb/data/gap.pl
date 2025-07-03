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
:- dynamic gap/1
gap() :- fail.



%% Predicates

% An XML file implementing a behavior tree of a heuristic used to make a decision.
%
% Assume file path is relative to the `working_directory` parameter of the
% running `assemble_decision_heuristic` ROS node.
%
% heuristic_of(?XMLfile, ?Gap)
:- dynamic heuristic_of/2

% A YAML file describing a configuration of ROS parameters corresponding to a particular heuristic.
%
% Assume file path is relative to the `working_directory` parameter of the
% running `assemble_decision_heuristic` ROS node.
%
% config_of(?YAMLfile, ?Gap)
:- dynamic config_of/2

% A condition to be implemented for each specific gap on the class of possible
% alternatives which would close the gap.
%
% alternative_of(?Alternative, ?Gap)
:- dynamic alternative_of/2

% A condition to be implemented for each specific gap on the set of possible
% cues which could be used to evaluate the alternatives of the gap.
% Assume every cue is some ROS service.
%
% cue_of(?RosService, ?Gap)
:- dynamic cue_of/2

% When a decision is made, the gap is closed with all members of the chosen set of alternatives.
%
% closed_with(?Gap, ?Alternative)
:- dynamic closed_with/2

% When an alternative is matched with the `alternative_of/2` predicate, it should
% be marked as `fetched_for` a particular gap.
%
% fetched_for(?Alternative, ?Gap)
:- dynamic fetched_for/2



%% Functions

% Copy an existing gap, creating a new one with a uuid but using the same
% heuristic, config, alternative class, and cues.
% Notably not copied are the `closed_with` and `fetched_for` predicates.
%
% duplicate_gap(--NewGap, ++OldGap)
duplicate_gap(Gnew, Gold) :-
    gap(Gold), uuid(Gnew), assertz((gap(Gnew))), 
    assertz((alternative_of(A, Gnew) :- alternative_of(A, Gold))),
    assertz((cue_of(C, Gnew) :- cue_of(C, Gold))),
    assertz((heuristic_of(Xml_file, Gnew) :- heuristic_of(Xml_file, Gold))),
    assertz((config_of(Yaml_file, Gnew) :- config_of(Yaml_file, Gold))),


