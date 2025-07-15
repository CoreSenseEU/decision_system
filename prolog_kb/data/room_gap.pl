%%% Hand-made definitions (and predicates used as cues) for room-gap
:- consult("gap.pl").

gap(room_gap).

%% alternatives
alternative_of(A, room_gap) :-
    room(A).

%% cues
available_for('/visited', room_gap).
available_for('/habits', room_gap).
available_for('/doorway_status', room_gap).
available_for('/distance', room_gap).


%%% Room Cue cues (used to decide which cue to use next in ttb)
% NOTE: validity of < 0.5 means the cues are invalid and should not be used
validity('/visited', 0.8).
validity('/habits', 0.7).
validity('/doorway_status', 0.6).
validity('/distance', 0.6).





