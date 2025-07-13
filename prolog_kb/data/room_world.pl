%%% World model for `room` decision
:- consult("shared_world.pl").

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
    \+ (dirty_tableware_found, clean_tableware_found).
room_likely_from_habits(R, 0) :-
    room(R), \+ room_likely_from_habits(R, 1).


% Doorway blocked/open
:- dynamic doorway_open/1.
:- dynamic doorway_blocked/1.

room_favorable_if_doorway_open(R, 0) :-
    room(R), doorway_blocked(R).
room_favorable_if_doorway_open(R, 1) :-
    room(R), doorway_open(R).
room_favorable_if_doorway_open(R, 0.5) :- %% state of the doorways is unknown
    room(R), \+ (doorway_open(R); doorway_blocked(R)).







