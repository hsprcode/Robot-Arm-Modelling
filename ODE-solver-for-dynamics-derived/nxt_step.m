function [q_iplus1,qdot_iplus1] = nxt_step(q_i,qdot_i,B,C,G,U,J,he,dt)
% gets the inputs below and gives q_i+1 and qdot_i+1 by
% solving the equations of motion ODEs.

%INPUTS for the system
% q_i = [1 1];
% qdot_i = [1 1];
% B = [1 5
%      0 2];
% C = eye(2);
% G = ones(2,1);
% U = ones(2,1);
% J = eye(2);
% he = ones(2,1);
% dt = 0.1;

% MAIN FUNCTION
n = (size(q_i,2)); 
% syms q(t) [1 n];
% q(t) = [q1; q2];
syms q(t) [1 n];
temp = [q(t)];
assume(temp,'real');
q(t) = temp';

% q1 = 1;
% q1dot  = 1;
% q2  = 1;
% q2dot = 1;
% qandqdotvector = [q1 q1dot q2 q2dot]; %this is the initial condition

% q_i = [1 1];
% qdot_i = [1 1];
% qandqdotvector = [q_i(1) qdot_i(1) q_i(2) qdot_i(2)];
pos_in_q = 1;
for vec_pos = 1:2:(size(q_i,2)+size(qdot_i,2))
    qandqdotvector(vec_pos) = q_i(pos_in_q);
    qandqdotvector(vec_pos+1) = qdot_i(pos_in_q);
    pos_in_q = pos_in_q+1;
end

[V] = odeToVectorField(B*diff(q, 2)+C*diff(q)+G == U - inv(J)*he);
M = matlabFunction(V,'vars', {'t','Y'})
sol = ode45(M,[0 dt],qandqdotvector);

fplot(@(x)deval(sol,x,1), [0, dt])
hold on;
fplot(@(x)deval(sol,x,2), [0, dt])
hold on;
fplot(@(x)deval(sol,x,3), [0, dt])
hold on;
fplot(@(x)deval(sol,x,4), [0, dt])

% q1plus1 = sol.y(1,end)
% q1dotplus1 = sol.y(2,end)
% q2plus1 = sol.y(3,end)
% q2dotplus1 = sol.y(4,end)

qandqdot_plus1_vector = sol.y(:,end);

pos_in_qplus1 = 1;
for vec_pos = 1:2:(size(q_i,2)+size(qdot_i,2))
    q_iplus1(pos_in_qplus1) = qandqdot_plus1_vector(vec_pos);
    qdot_iplus1(pos_in_qplus1) = qandqdot_plus1_vector(vec_pos+1);
    pos_in_qplus1 = pos_in_qplus1+1;
end

%OUTPUTS
% q_iplus1
% qdot_iplus1
end
