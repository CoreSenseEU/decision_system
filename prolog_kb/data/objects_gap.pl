%%% Hand-made definitions (and predicates used as cues) for objects_gap
:- consult("gap.pl").

gap(objects_gap).

%% alternatives
alternative_of(A, objects_gap) :-
    object(A).

%% cues
cue_of('/visited', objects_gap).
cue_of('/distance', objects_gap).
cue_of('/is_book', objects_gap).
cue_of('/in_doorway', objects_gap).

%% runtime
heuristic_of(Path, objects_gap) :- 
    ament_package_share_prefix(Dir), 
    atom_concat(Dir, 'behavior_trees/decision_heuristics/decide_on_objects.xml', Path).
entry_point_of('DecideOnGap_objects_gap', objects_gap).
config_of(Path, objects_gap) :- 
    ament_package_share_prefix(Dir), 
    atom_concat(Dir, 'config/decision_heuristics/decide_on_objects.yaml', Path).

