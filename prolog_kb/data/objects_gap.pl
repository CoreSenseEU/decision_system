%%% Hand-made definitions (and predicates used as cues) for objects_gap
:- consult("gap.pl").

gap(objects_gap).

%% alternatives
alternative_of(A, objects_gap) :-
    object(A).

%% cues
% Assume all available cues are relevant for this heursitic...because it is not iterative.
cue_of(C, objects_gap) :- available_for(C, objects_gap).

available_for('/visited', objects_gap).
available_for('/distance', objects_gap).
available_for('/is_book', objects_gap).
available_for('/in_doorway', objects_gap).

%% runtime
heuristic_of(Path, objects_gap) :- 
    ament_package_share_prefix(Dir), 
    atom_concat(Dir, 'behavior_trees/decision_heuristics/decide_on_objects.xml', Path).
entry_point_of('DecideOnGap_objects_gap', objects_gap).
config_of(Path, objects_gap) :- 
    ament_package_share_prefix(Dir), 
    atom_concat(Dir, 'config/decision_heuristics/decide_on_objects.yaml', Path).

