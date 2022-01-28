addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/10; % Note that we choose a larger Ts here to speed up the simulation
rocket = Rocket(Ts);
H = 4;
nmpc = NMPC_Control(rocket, H);

% MPC reference with default maximum roll = 15 deg
Tf = 30;
ref = @(t , x_) rocket.MPC_ref(t , Tf);

% MPC reference with specified maximum roll = 50 deg
roll max = deg2rad(50);
ref = @(t_ , x_) rocket.MPC_ref(t_, Tf, roll_max);
[T, X, U, Ref] = rocket.simulate_f(x0, Tf, nmpc, ref);

