function [F, M, trpy, drpy] = lqr_controller(qd, t, qn, params, trajhandle)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================
persistent gd;
persistent icnt;

 if isempty(gd)
     gd = zeros(0,3);
     icnt = 0;
 end
 icnt = icnt + 1;

%% Parameter Initialization
if ~isempty(t)
desired_state = trajhandle(t, qn);

end

phi   = qd{qn}.euler(1);
theta = qd{qn}.euler(2);
psi   = qd{qn}.euler(3);
p     = qd{qn}.omega(1);
q     = qd{qn}.omega(2);
r     = qd{qn}.omega(3);

X(1)  = qd{qn}.pos(1);   %x
X(2)  = qd{qn}.pos(2);   %y
X(3)  = qd{qn}.pos(3);   %z
X(4)  = qd{qn}.vel(1);   %xdot
X(5)  = qd{qn}.vel(2);   %ydot
X(6)  = qd{qn}.vel(3);   %zdot
X(7)  = phi;             %phi
X(8)  = theta;           %theta
X(9)  = psi;             %psi
X(10) = p;               %p
X(11) = q;               %q
X(12) = r;               %r
X = X(:);

X_des(1) = desired_state.pos(1);  %x_des
X_des(2) = desired_state.pos(2);  %y_des
X_des(3) = desired_state.pos(3);  %z_des
X_des(4) = desired_state.vel(1);  %xdot_des
X_des(5) = desired_state.vel(2);  %ydot_des
X_des(6) = desired_state.vel(3);  %zdot_des
X_des(7) = 0;                     %phi_des
X_des(8) = 0;                     %theta_des
X_des(9) = desired_state.yaw;     %yaw_des
X_des(10) = 0;                    %p_des
X_des(11) = 0;                    %q_des
X_des(12) = desired_state.yawdot; %r_des
X_des = X_des(:);

g      = params.grav;
q_mass = params.mass;
Ixx    = params.I(1,1);
Iyy    = params.I(2,2);
Izz    = params.I(3,3);
%%%%%%%Linearize the function %%%%%%%
A = [0  0  0  1  0  0  0                                                    0                                       0                                  0                    0    0;
     0  0  0  0  1  0  0                                                    0                                       0                                  0                    0    0;
     0  0  0  0  0  1  0                                                    0                                       0                                  0                    0    0;
     0  0  0  0  0  0  g*sin(psi)                                           g*cos(psi)                              g*(phi*cos(psi) - theta*sin(psi))  0                    0    0;
     0  0  0  0  0  0 -g*cos(psi)                                           g*sin(psi)                              g*(phi*sin(psi) + theta*cos(psi))  0                    0    0;
     0  0  0  0  0  0  0                                                    0                                       0                                  0                    0    0;
     0  0  0  0  0  0  0                                                   -p*sin(theta) + r*cos(theta)             0                                  cos(theta)           0    sin(theta);
     0  0  0  0  0  0  (p*sin(theta) - r*cos(theta))*(sec(phi)^2)           (p*cos(theta) + r*sin(theta))*tan(phi)  0                                  sin(theta)*tan(phi)  1    -cos(theta)*tan(phi);
     0  0  0  0  0  0  (r*cos(theta) - p*sin(theta))*sin(phi)/(cos(phi)^2) -(r*sin(theta) + p*cos(theta))/cos(phi)  0                                 -sin(theta)/cos(phi)  0    cos(theta)/cos(phi);
     0  0  0  0  0  0  0                                                    0                                       0                                  0                    0    0;
     0  0  0  0  0  0  0                                                    0                                       0                                  0                    0    0;
     0  0  0  0  0  0  0                                                    0                                       0                                  0                    0    0];

B = [0         0      0      0;
     0         0      0      0;
     0         0      0      0;
     0         0      0      0;
     0         0      0      0;
     1/q_mass  0      0      0;
     0         0      0      0;
     0         0      0      0;
     0         0      0      0;
     0         1/Ixx  0      0;
     0         0      1/Iyy  0;
     0         0      0      1/Izz];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Calculate the LQR control gain
Q = diag([20,20,20,1,1,1,0.01,0.01,0.01,0.01,0.01,0.01]);
R = diag([0.001,0.1,0.1,0.1]);
K = lqr(A,B,Q,R);

%calculate the errors
eX     = X-X_des;
eU     = -K*eX;
%%%%%%%%%%%%
F = eU(1);
M = [eU(2);eU(3);eU(4)];
%%%%%%%%%%%%
%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0,       0,         0];

end
