%%% Hand-made definitions (and predicates used as cues) for room-gap

gap(room_gap)

%% alternatives
alternative_of(A, room_gap) :-
    room(A).

%% cues
cue_of('/visited', room_gap).
cue_of('/habits', room_gap).
cue_of('/doorway_status', room_gap).
cue_of('/distance', room_gap).





