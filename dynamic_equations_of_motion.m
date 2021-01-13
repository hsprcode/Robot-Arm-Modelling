% DEFINE THE NUMBER OF JOINTS IN LINE-5 =, AND DH PARAMETERS IN LINE-212
% FOR YOUR ROBOT CONFIGURATION 
% (change the translation vector for each link, defined as p in LINE-39)
% number of joints
n=3;

% symbolic variables
syms th [1 n];% thetai
assume(th,'real');
syms alp [1 n];% alphai
assume(alp,'real');
syms a [1 n];% ai s
assume(a,'real');
syms d [1 n];% di s
assume(d,'real');
syms Kr [1 n];
assume(Kr,'real');

% T(1) = [cos(th(1))]
% diff(T,th(1))

%T(:,:,i) is Transformation fron base 0 to i
T00 = eye(4);
T(:,:,1) = [cos(th(1)) -(sin(th(1)))*(cos(alp(1)))  (sin(th(1)))*(sin(alp(1))) (a(1))*(cos(th(1)));
            sin(th(1))  (cos(th(1)))*(cos(alp(1))) -(cos(th(1)))*(sin(alp(1))) (a(1))*(sin(th(1)));
            0           sin(alp(1))                 cos(alp(1))                 d(1);
            0           0                           0                           1];
for i = 2:1:n
T(:,:,i) = T(:,:,i-1)*[cos(th(i)) -(sin(th(i)))*(cos(alp(i)))  (sin(th(i)))*(sin(alp(i))) (a(i))*(cos(th(i)));
                       sin(th(i))  (cos(th(i)))*(cos(alp(i))) -(cos(th(i)))*(sin(alp(i))) (a(i))*(sin(th(i)));
                       0           sin(alp(i))                 cos(alp(i))                 d(i);
                       0           0                           0                           1];
end

% z axis; and translation vector(assumed at the base - change accordingly)
 p0 = [0 0 0]';
 z0 = [0 0 1]'; 
 for i = 1:1:n
     p(:,:,i) = T([1 2 3],4,i);
     z(:,:,i) = T([1 2 3],3,i);
 end

% joint type defenitions
% sigma is 0 for revolute, 1 for prismatic
sigma = [0 1 1]; %round(rand(1,n)); - USER DEFINED
% Jp_l(:,:,i) initialization and definition - JACOBIAN 1
for i = 1:1:n
    Jp_l(:,:,i) = sym(zeros(3,n));
end
for i = 1:1:n
    for j = 1:1:n
        if sigma(j) == 1
            if j<= i
                if j == 1
                    Jp_l(:,j,i) = z0;
                else
                    Jp_l(:,j,i) = z(:,:,j-1);
                end
            else
                Jp_l(:,j,i) = [0 0 0]';
            end
                
        elseif sigma(j) == 0
            if j<= i
                if j == 1
                    Jp_l(:,j,i) = cross(z0,(p(:,:,i)-p0));
                else
                    Jp_l(:,j,i) = cross(z(:,:,j-1),(p(:,:,i)-p(:,:,j-1)));
                end
            else
                Jp_l(:,j,i) = [0 0 0]';
            end
        end
    end
end

% Jo_l(:,:,i) initialization and definition
for i = 1:1:n
    Jo_l(:,:,i) = sym(zeros(3,n));
end
for i = 1:1:n
    for j = 1:1:n
        if sigma(j) == 1
            Jo_l(:,j,i) = [0 0 0]';
                
        elseif sigma(j) == 0
            if j<= i
                if j == 1
                    Jo_l(:,j,i) = z0;
                else
                    Jo_l(:,j,i) = z(:,:,j-1);
                end
            else
                Jo_l(:,j,i) = [0 0 0]';
            end
        end
    end
end
    
% Jp_m(:,:,i) initialization and definition
for i = 1:1:n
    Jp_m(:,:,i) = sym(zeros(3,n));
end
for i = 1:1:n
    for j = 1:1:n
        if sigma(j) == 1
            if j<= i
                if j == 1
                    Jp_m(:,j,i) = z0;
                else
                    Jp_m(:,j,i) = z(:,:,j-1);
                end
            else
                Jp_m(:,j,i) = [0 0 0]';
            end
                
        elseif sigma(j) == 0
            if j<= i
                if j == 1
                    Jp_m(:,j,i) = cross(z0,(p0-p0));
                else
                    Jp_m(:,j,i) = cross(z(:,:,j-1),(p(:,:,i-1)-p(:,:,j-1)));
                end
            else
                Jp_m(:,j,i) = [0 0 0]';
            end
        end
    end
end

% Jo_m(:,:,i) initialization and definition
for i = 1:1:n
    Jo_m(:,:,i) = sym(zeros(3,n));
end
for i = 1:1:n
    for j = 1:1:n
        if j<= i-1
            Jo_m(:,j,i) = Jo_l(:,j,i);
        elseif j == i
            if i == 1
                Jo_m(:,j,i) = Kr(i)*z0;
            else
                Jo_m(:,j,i) = Kr(i)*z(:,:,i-1);
            end
        end
    end
end

syms m_l [1 n]; %mass of links
assume(m_l,'real');
syms I_l [1 n]; %Inertia of links
assume(I_l,'real');
syms m_m [1 n]; %mass of the motors
assume(m_m,'real');
syms I_m [1 n]; %inertia of the motors
assume(I_m,'real');

%B MATRIX
B(:,:) = zeros(n,n);
for i = 1:n
    B = B + m_l(i)*Jp_l(:,:,i)'*Jp_l(:,:,i) + Jo_l(:,:,i)'*I_l(i)*Jo_l(:,:,i) + m_m(i)*Jp_m(:,:,i)'*Jp_m(:,:,i) + Jo_m(:,:,i)'*I_m(i)*Jo_m(:,:,i);
end
%simplify(subs(B))

%defining qdot
syms Q(t) [1 n];
qdot = diff(Q(t),t); %[qdot(1) ... ]'
assume(qdot,'real');

%defining q
for i = 1:1:n
    if sigma(i) == 0
        q(i) = th(i);
    elseif sigma(i) == 1
        q(i) = d(i);
    end
end
% C MATRIX
C(:,:) = sym(zeros(n,n));
for i = 1:1:n
    for j = 1:1:n
        for k = 1:1:n
            c(i,j,k) = 0.5*(diff(B(i,j),q(k)) + diff(B(i,k),q(j)) - diff(B(j,k),q(i)))*qdot(k);
            C(i,j) = C(i,j)+c(i,j,k);
        end
    end
end

syms g [1 n];
assume(g,'real');
syms gc; %gc = 9.81;
assume(gc,'real');
g0 = [0 0 -gc]';

for i = 1:1:n
    g(i) = 0;
end
for i = 1:1:n
    for j = 1:1:n
        g(i) = g(i) - (m_l(j)*g0'*Jp_l(:,i,j)) - (m_m(j)*g0'*Jp_m(:,i,j));
    end
end
G = g';

%defining qddot
syms qddot [1 n];
assume(qddot,'real');

TAU = B*qddot' + C*qdot' + G;

%%%%%%%%%%%%%%%%%%%
% % % % DHvalues
a1 = 0; alp1 = 0; d1 = 0;
a2 = 0; alp2 = -pi/2; th2 = 0;
a3 = 0; alp3 = 0; th3 = 0;
%%%%%%%%%%%%%%%%%%%%

TAU = simplify(subs(TAU));
syms tau_ [1 n];
assume(tau_,'real');

for i = 1:1:n
    disp(['Equation ' num2str(i)]);
    disp([tau_(i)]);
    disp('=');
    disp([(TAU(i))]);
end







            

