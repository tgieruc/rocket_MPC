addpath(fullfile('..', 'src')); 

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20; % Sample time
H  = 2; % Horizon length in seconds
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


%% X 
% Get control input
x0 = [0, 0 ,0 ,0]'; % (Assign appropriately)
x_ref = -5; % (Assign appropriately)
[T, X_sub, U_sub] = rocket.simulate(sys_x, x0, Tf, @mpc_x.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, x_ref);

%% Y
x0 = [0, 0 ,0 ,0]'; % (Assign appropriately)
x_ref = -5; % (Assign appropriately)
[T, X_sub, U_sub] = rocket.simulate(sys_y, x0, Tf, @mpc_y.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us, x_ref);

%% Z
x0 = [0, 0]'; % (Assign appropriately)
x_ref = -5; % (Assign appropriately)
[T, X_sub, U_sub] = rocket.simulate(sys_z, x0, Tf, @mpc_z.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, x_ref);

%% Roll
x0 = [0, 0]'; % (Assign appropriately)
x_ref = pi/4; % (Assign appropriately)
[T, X_sub, U_sub] = rocket.simulate(sys_roll, x0, Tf, @mpc_roll.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, x_ref);
