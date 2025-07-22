%%
clc;clear all;close all
syms m  Fx Fy Fz I
syms rx ry rz

r = [rx;ry;rz]
F = [Fx;Fy;Fz]
eq1 = cross(r, F)

deq1 = jacobian(eq1, F)
% deq1 = hessian(eq1, F)

%%
clc; clear all; close all;
syms roll pitch yaw
syms droll dpitch dyaw
syms Rx Ry Rz
syms wx wy wz

Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1]
Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)]
Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)]

R = Rz * Ry * Rx

dR = diff(R, roll) * droll + diff(R, pitch) * dpitch + diff(R, yaw) * dyaw

ss = dR*transpose(R) 

w = [ss(3,2); ss(1,3); ss(2,1)]
A = jacobian(simplify(w), [droll, dpitch, dyaw])


%%
clc; clear all; close all;
PARA = PARA;

H = PARA.H

% state
   theta = sym("theta"  , [3, 1]);
     COM = sym("COM"    , [3, 1]); 
       w = sym("w"      , [3, 1]);
    dCOM = sym("dCOM"   , [3, 1]);
 contact = sym("contact", [3, 1]);

% input

% rL = sym("rL", [3, 1]);
% rR = sym("rR", [3, 1]);

mL = sym("mL", [3, 1]);
fL = sym("fL", [3, 1]);
mR = sym("mR", [3, 1]);
fR = sym("fR", [3, 1]);

% stepping vector
delcontactL = sym("delcontactL", [3, 1]);
delcontactR = sym("delcontactR", [3, 1]);

% etc
syms M
     ag = [0; 0; -PARA.g]
  I_inv = sym("I_inv", [3, 3])
     rL = contact + delcontactL - COM
     rR = contact + delcontactR - COM 
     % rL = contact - COM
     % rR = contact - COM

     Rf_inv = sym("Rf_inv", [3, 3])

% state, input, stepping vector (variables of function fx)
         x = [theta; COM; w; dCOM; contact]
         u = [mL; fL; mR; fR]
delcontact = [delcontactL; delcontactR]

% continous state equation
    ddCOM = (1/M) .* (fL + fR) + ag
       dw = I_inv * (cross(rL, fL) + cross(rR, fR) + mL + mR)
 dcontact = zeros(3, 1)
   dtheta = Rf_inv * w

       dx = [dtheta; dCOM; dw; ddCOM; dcontact]

% coefficient
A = jacobian(dx, x)
B = jacobian(dx, u)
C = jacobian(dx, delcontact)




















% test = I_inv * skew(rL)
% B_test = B(7:9, 4:6)


% A_test = A(7:9, 4:6)
% test_A = I_inv * skew(fL+fR)

% A_last_three = A(:, end-2:end)
C(:, 1:3)
test_C = - I_inv * (skew(fL)+skew(fR))