%%% World model for `room` decision

room(office).
room(bedroom).
room(living_room).
room(kitchen).

:- dynamic dirty_tableware_found/0.
:- dynamic clean_tableware_found/0.

room_likely_from_habits(bedroom, 1) :-
    clean_tableware_found.
room_likely_from_habits(bedroom, 1) :-
    dirty_tableware_found.
room_likely_from_habits(office, 1) :-
    \+ (dirty_tableware_present, clean_tableware_found).
room_likely_from_habits(R, 0) :-
    room(R), \+ room_likely_from_habits(R, 1).


% Has a room been visited (assume we looked at all the objects there)
%
% visited(?Room)
:- dynamic visited/1.

room_unlikely_if_visited(R, 0) :-
    room(R), visited(R).
room_unlikely_if_visited(R, 1) :-
    room(R), \+ visited(R).


% Doorway blocked/open
:- dynamic doorway_open/1.
:- dynamic doorway_blocked/1.

room_favorable_if_doorway_open(R, 0) :-
    room(R), doorway_blocked(R).
room_favorable_if_doorway_open(R, 1) :-
    room(R), doorway_open(R).
room_favorable_if_doorway_open(R, 0.5) :- %% state of the doorways is unknown
    room(R), \+ (doorway_open(R); doorway_blocked(R)).


% Distance to room
:- dynamic neg_distance_to_room/2.

% TODO: match real values from sim. For now they are randomized and negated so bigger is worse
neg_distance_to_room(R, S) :-
    room(R), randon(N), S = -10 * N.





