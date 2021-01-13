%% INPUTS for the system 

q_i = [1 1];
qdot_i = [1 1];

% B, C, and G can be obtained using the dynamic_equations_of_motion code
B = [1 5
     0 2];
C = eye(2);
G = ones(2,1);

% U is the input
U = ones(2,1);

% J is the Jacobian
J = eye(2);

% he is the end effector forces
he = ones(2,1);

% dt is the step time
dt = 0.1;

%% gets the inputs above and gives q_i+1 and qdot_i+1 by solving the ODE
[q_iplus1,qdot_iplus1] = nxt_step(q_i,qdot_i,B,C,G,U,J,he,dt)