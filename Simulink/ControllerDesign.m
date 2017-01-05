clc
% clear all

syms x y z roll pitch yaw u v w wx wy wz t...
    g m Ix Iy Iz d c1 c2 c3 c4 d1 d2 d3 d4 ...
    T taux tauy tauz dT ddT T0...
    

%% rotation matrix from eA = R(lambda)*eB
Rr = [1 0      0;
    0 cos(roll) -sin(roll);
    0 sin(roll) cos(roll)];
Rp = [cos(pitch)  0 sin(pitch);
    0       1 0;
    -sin(pitch) 0 cos(pitch)];
Ry = [cos(yaw) -sin(yaw) 0;
    sin(yaw)  cos(yaw) 0;
    0       0      1];

R = Ry*Rp*Rr;

%% Q
Q = [1 sin(roll)*tan(pitch) cos(roll)*tan(pitch);
    0 cos(roll) -sin(roll);
    0 sin(roll)*sec(pitch) cos(roll)*sec(pitch)];

%% Equations of motion, 
e3 = [0 0 1].';

% states
X = [x y z roll pitch yaw u v w wx wy wz].';
% inputs
U = [T taux tauy tauz].';

% non linear equations of motion
F = simplify([u; v; w;
    Q*[wx;wy;wz];
    1/m*(-R*T*e3  + m*g*e3);  
    inv(diag([Ix, Iy, Iz]))*[taux;tauy;tauz]] );
G = X;

%% select equations of interest
G = [x, y, z, yaw, u, v];

%% linearization
A = simplify(jacobian(F.',X));
B = simplify(jacobian(F.',U));
C = simplify(jacobian(G.',X));
D = simplify(jacobian(G.',U));

% evaluate around: roll = pitch = yaw = wx = wy = wz = 0
roll = 0;
pitch = 0;
yaw = 0;
wx = 0;
wy = 0;
wz = 0;
T = T0;

A = simplify(eval(A));
B = simplify(eval(B));
C = (eval(C));
D = (eval(D));

% evaluate for all parameters
l = 0.140; % [m] distance quad CM to stand joint (not used anymore)

parameters; % run script
g = par.g;
m = par.m;
Ix = par.Ix;
Iy = par.Iy;
Iz = par.Iz;
d = par.d;
k = par.k;
cmean = par.cmean;
c_sdv = par.c_sdv;
dmean = par.dmean;
d_sdv = par.d_sdv;
tau_c = par.tau_c;

T0 = m*g;

c1 = cmean; c2 = cmean; c3 = cmean; c4 = cmean;
d1 = dmean; d2 = dmean; d3 = dmean; d4 = dmean;

A = eval(A)
B = eval(B)

% from moment inputs to pwmbar inputs, Thrust = c1*(pwm + k)^2 = c1*pwmbar
pwmbarToTau = [c1 c2 c3 c4;
    d*c1 -d*c2 -d*c3 d*c4;
    d*c1 d*c2 -d*c3 -d*c4;
    -d1 d2 -d3 d4]
% tau to pwm
tauToPwmbar = inv(pwmbarToTau);

%% observer for height (Z) and derivative of height (W)
% x = [z, w] 
Ah = [0 1; 0 0];
Ch = [1 0];
Ph = [-8, -4];
Lh = place(Ah',Ch',Ph);
Lh = Lh'

%% observer for X-Y position and derivative of X-Y position (U-V)
% x = [X, U] ( for Y position use the same gain)
Axy = [0 1; 0 0];
Cxy = [1 0];
Pxy = [-16, -8];
Lxy = place(Axy',Cxy',Pxy);
Lxy = Lxy'

%% observer for roll pitch 
% x = [Roll, dRollU] ( for Pitch position use the same gain)
Arp = [0 1; 0 0];
Crp = [1 0; 0 1];
Prp = [-0.01, -90];
Lrp = place(Arp',Crp',Prp);
Lrp = Lrp'


