clc; clear;
% Harishankar Prabhakaran
disp(['COMBINING SERIES OF HOMOGENEOUS TRANSFORMATIONS fOR A ROBOT']);
tot_n = input('Total number of homogeneous transformations: ');
for m = 1:1:tot_n
    fprintf('\n')
    disp(['Transformation from frame ' (num2str(m)-1) ' to frame ' num2str(m) ' - Info needed']);
    symb_r_numb = input('Is the matrix symbolic or numerical? enter sym or num: ', 's');
    fprintf('\n')
    if symb_r_numb == 'sym' 
        flag(m) = 0;
        r(m) = input(['Rotation for this matrix ' num2str(m) ' is about? enter x or y or z: '],'s');
        if isempty(r(m))
            aS(m) = 0;
        else
            aS(m) = input('and for what symbolic angle?: (enter a single letter, eg: w) ','s');
        end
        trnsS_x(m) = input('enter the symbolic translation in the x direction? (eg: a) ','s');
        trnsS_y(m) = input('enter the symbolic translation in the y direction? (eg: b) ','s');
        trnsS_z(m) = input('enter the symbolic translation in the z direction? (eg: c) ','s');
    elseif symb_r_numb == 'num'
        flag(m) = 1;
        r(m) = input(['Rotation for this matrix ' num2str(m) ' is about? enter x or y or z: '],'s');
        if isempty(r(m))
            aS(m) = 0;
        else
            a(m) = input('for what numerical angle? (in radians; eg: pi): ');
        end
        trns_x(m) = input('enter the numerical translation in the x? ');
        trns_y(m) = input('enter the numerical translation in the y? ');
        trns_z(m) = input('enter the numerical translation in the z? ');
    end
end

syms X(T) Y(T) Z(T);
X(T) = [1 0 0;
     0 cos(T) -sin(T);
     0 sin(T) cos(T)];
 
Y(T) = [cos(T) 0 sin(T);
     0 1 0;
    -sin(T) 0 cos(T)];

Z(T) = [cos(T) -sin(T) 0;
     sin(T) cos(T) 0;
     0 0 1];

for n = 1:1:tot_n
    if flag(n) == 1
        if r(n) == 'x'
            k(:,:,n) = [X(a(n)), [trns_x(n), trns_y(n), trns_z(n)]';
                        0 0 0 1];
        elseif r(n) == 'y'
            k(:,:,n) = [Y(a(n)), [trns_x(n), trns_y(n), trns_z(n)]';
                        0 0 0 1];
        elseif r(n) == 'z'
            k(:,:,n) = [Z(a(n)), [trns_x(n), trns_y(n), trns_z(n)]';
                        0 0 0 1];          
        end
    elseif flag(n) == 0
        if r(n) == 'x'
            k(:,:,n) = [X(aS(n)), [trnsS_x(n), trnsS_y(n), trnsS_z(n)]';
                        0 0 0 1];
        elseif r(n) == 'y'
            k(:,:,n) = [Y(aS(n)), [trnsS_x(n), trnsS_y(n), trnsS_z(n)]';
                        0 0 0 1];
        elseif r(n) == 'z'
            k(:,:,n) = [Z(aS(n)), [trnsS_x(n), trnsS_y(n), trnsS_z(n)]';
                        0 0 0 1];          
        end
    end
end


k
fnl_trnsfrm = eye(4);
for n = 1:1:tot_n
    fnl_trnsfrm = fnl_trnsfrm*k(:,:,n);
end
disp(['THE NET HOMOGENEOUS TRANSFORMATION IS BELOW']);
T_0n = fnl_trnsfrm

% disp(['THE INVERSE OF THE HOMOGENEOUS TRANSFORMATION']);
% R_0n = T_0n(1:3,1:3); P_0n = T_0n(1:3,4);
% T_n0 = [R_0n' -R_0n'*P_0n; 
%         0 0 0 1]


