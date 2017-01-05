% Body frame (B)
%                              /<- d ->/
% 
%                           xB (Front)
%                          /  
%                         /  
%       Prop1    /->     /      <-/   Prop2
%               O       /        O    
%            <-/       /        /->
%                     /
%                CM: O  - - - - - - - - - - > yB (Right)
%                    |
%        <-/         |      /->  
%         O          |     O
% Prop4  /->         |  <-/   Prop3
%                    |
%                    |
%                    |
%                    V zB (Bottom)
%                       

% Inertia frame (A)
%                           xA (North)
%                          /  
%                         /  
%                        /           
%                       /            
%                      /            
%                     /
%                     - - - - - - - - - - > yB (East)
%                    |
%                    |   
%                    |   
%                    |  
%                    |
%                    |
%                    |
%                    V zB (Depth / ground)

% par
par.g =9.81;            % gravitational acceleration
par.m = 0.456;          % quadrotor mass
par.Ix = 0.0033;        % moment of inertia over quad x axis
par.Iy = 0.0037;        % moment of inertia over quad y axis
par.Iz = 0.0068;        % moment of inertia over quad z axis
par.d = 0.126;          % distance between axis and propeller center (diagonal to orgin is thus sqrt(2*d^2)) 

% thrust model: T_i = c_i*(pwm_i + k)^2
par.k = 25.19;          % offset pwm signal
par.cmean = 0.1318e-3;  % average thrust coefficient
par.c_sdv = 1.349e-5;   % standard deviation thrust coefficient

% torque model: T_i = c_i*(pwm_i + k)^2
par.dmean = 0.8722e-5;  % average torque coefficient
par.d_sdv = 1.039e-6;   % standard deviation torque coefficient
% (note: this is the absolute value, multiply with -1 for motor 2 and 4)

par.tau_c = 0.05;        % time constant thrust/torque build up

% %% indoor hull plus Aruco markers
% par.m = 0.502;          % quadrotor mass
% par.Ix = 0.0057;        % moment of inertia over quad x axis
% par.Iy = 0.0058;        % moment of inertia over quad y axis
% par.Iz = 0.0117;        % moment of inertia over quad z axis

% put in vector form for Simulink
parVec = [par.g;
par.m;
par.Ix;
par.Iy;
par.Iz;
par.d;
par.k;
par.cmean;
par.c_sdv;
par.dmean;
par.d_sdv;
par.tau_c];

par_m = par.m;
par_g = par.g;