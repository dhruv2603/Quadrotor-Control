function [F, M, trpy, drpy] = pid_controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

persistent gd;
persistent icnt;
persistent norme;
 if isempty(gd)
     gd = zeros(0,3);
     icnt = 0;
     norme = 0;
 end
 icnt = icnt + 1;

% =================== Your code starts here ===================
%% Parameter Initialization
pos_error = qd{qn}.pos_des - qd{qn}.pos;
vel_error = qd{qn}.vel_des - qd{qn}.vel;

theta_dot_des = 0;
phi_dot_des   = 0;

% Kp = [5.5;4.5;10];
% Kd = [1.9;2.5;7.4];
% Kp_phi   = 60;
% Kd_phi   = 9;
% Kp_theta = 48;
% Kd_theta = 8;
% Kp_psi   = 20;
% Kd_psi   = 0.5;

Kp = [4.8;4.4;2.8];
Kd = [2.3;2.3;3];
Kp_phi   = 39;
Kd_phi   = 8;
Kp_theta = 39;
Kd_theta = 10;
Kp_psi   = 20;
Kd_psi   = 10;

if(mod(icnt,5) == 1)
    gd   = qd{qn}.acc_des + Kd .* vel_error + Kp .* pos_error;
end
acc = gd;


phi   = (1/params.grav) * (acc(1) * sin(qd{qn}.yaw_des) - acc(2) * cos(qd{qn}.yaw_des));
theta = (1/params.grav) * (acc(1) * cos(qd{qn}.yaw_des) + acc(2) * sin(qd{qn}.yaw_des));

phi_error   = phi            - qd{qn}.euler(1);
theta_error = theta          - qd{qn}.euler(2);
yaw_error   = qd{qn}.yaw_des - qd{qn}.euler(3);

dot_phi_error   = phi_dot_des       - qd{qn}.omega(1);
dot_theta_error = theta_dot_des     - qd{qn}.omega(2);
dot_yaw_error   = qd{qn}.yawdot_des - qd{qn}.omega(3);

norme = norme + norm(pos_error);

F = params.mass * acc(3) + params.mass * params.grav;  

M = params.I * [Kp_phi   * (phi_error)     + Kd_phi   * (dot_phi_error) ;
                Kp_theta * (theta_error)   + Kd_theta * (dot_theta_error);
                Kp_psi   * (yaw_error)     + Kd_psi   * (dot_yaw_error)];

disp('t:');
disp(t);
disp('Phi error:');
disp(phi_error);
% disp(phi_error);
disp('Theta error:');
disp(theta_error);
disp('Yaw error:');
disp(yaw_error);
disp('Position error:');
disp(pos_error);
disp('Velocity error:');
disp(vel_error);
disp(norme);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% =================== Your code ends here ===================

%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0, 0, 0];

end
