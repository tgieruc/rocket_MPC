addpath(fullfile('..', 'src')); 

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20; % Sample time
H  = 6; % Horizon length in seconds
Tf = 6.0;
rocket = Rocket(Ts);

[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% Design MPC controller
mpc_x = MPC_Control_x(sys_x, Ts, H);
mpc_y = MPC_Control_y(sys_y, Ts, H);
mpc_z = MPC_Control_z(sys_z, Ts, H);
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);


% % X 
% Get control input
x0 = [-0.00546807,0.086549  ,0.320912 ,1.17422]'; 
x_ref = 2.8483; 
[T, X_sub, U_sub] = rocket.simulate(sys_x, x0, Tf, @mpc_x.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, x_ref);


% % Y
% x0 = [-0.0883577, 0.0864673 ,-0.182421 ,-0.682468]'; 
% x_ref = -1.6445; 
% [T, X_sub, U_sub] = rocket.simulate(sys_y, x0, Tf, @mpc_y.get_u, x_ref);
% ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us, x_ref);

% %% Z
% x0 = [-0.0263204, 0.658281]'; 
% x_ref = 1.1971; 
% [T, X_sub, U_sub] = rocket.simulate(sys_z, x0, Tf, @mpc_z.get_u, x_ref);
% ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, x_ref);
% 
% % Roll
% x0 = [0.0361875, -0.259322]'; 
% x_ref = 0; 
% [T, X_sub, U_sub] = rocket.simulate(sys_roll, x0, Tf, @mpc_roll.get_u, x_ref);
% ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, x_ref);