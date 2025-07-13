% Modelling model
:- consult("gap.pl").

% Mission model
:- consult("room_gap.pl").
:- consult("objects_gap.pl").

% World model
:- consult("objects_world.pl").
:- consult("room_world.pl").

% Agent model
:- consult("agent.pl").
:- consult("decision-engines.pl").


% Stacked to demonstrate interesting results from dawe's rule and condorcet
in_doorway(object_1).
in_doorway(object_2).

visited(object_3).
visited(object_4).

confidence_is_book(object_1, 0.8).
confidence_is_book(object_2, 0.2).
confidence_is_book(object_3, 0.0).
confidence_is_book(object_4, 0.6).
confidence_is_book(object_5, 0.6).
confidence_is_book(object_6, 0.6).
