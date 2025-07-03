% World model
consult("room_world.pl").

% Agent model
consult("agent.pl")
consult("decision-engines.pl").

% Mission model
consult("room_gap.pl").

% Modelling model
consult("gap.pl").
