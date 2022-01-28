addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20; % Sample time
rocket = Rocket(Ts);

[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

%% X 
% Design MPC controller
H = 6; % Horizon length in seconds
mpc_x = MPC_Control_x(sys_x, Ts, H);
% Get control input
Tf = 6.0;
x0 = [0, 0 ,0 ,5]'; % (Assign appropriately)
[T, X_sub, U_sub] = rocket.simulate(sys_x, x0, Tf, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);

%% y
% Design MPC controller
H = 6; % Horizon length in seconds
mpc_y = MPC_Control_y(sys_y, Ts, H);
% Get control input
Tf = 6.0;
x0 = [0, 0 ,0 ,5]'; % (Assign appropriately)
[T, X_sub, U_sub] = rocket.simulate(sys_y, x0, Tf, @mpc_y.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);

%% z
% Design MPC controller
H = 6; % Horizon length in seconds
mpc_z = MPC_Control_z(sys_z, Ts, H);
% Get control input
Tf = 8.0;
x0 = [0, 5]'; % (Assign appropriately)
[T, X_sub, U_sub] = rocket.simulate(sys_z, x0, Tf, @mpc_z.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us);

%% Roll
% Design MPC controller
H = 6; % Horizon length in seconds
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);
% Get control input
Tf = 6.0;
x0 = [0, pi/4]'; % (Assign appropriately)
[T, X_sub, U_sub] = rocket.simulate(sys_roll, x0, Tf, @mpc_roll.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);

