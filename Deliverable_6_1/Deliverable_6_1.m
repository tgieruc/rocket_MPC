addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/10; % Note that we choose a larger Ts here to speed up the simulation
rocket = Rocket(Ts);
H = 4;
nmpc = NMPC_Control(rocket, H);

% MPC reference with default maximum roll = 15 deg
Tf = 30;
Simul_time = 30;
ref = @(t_, x_) rocket.MPC_ref(t_, Tf);
x0 = zeros(12,1);
[T, X_15, U_15, Ref_15] = rocket.simulate_f(x0, Simul_time, nmpc, ref);

% Plot pose
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X_15, U_15, Ref_15);
ph.fig.Name = 'NMPC_15'; % Set a figure title

% % MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref = @(t_, x_) rocket.MPC_ref(t_, Tf, roll_max);
[T, X_50, U_50, Ref_50] = rocket.simulate_f(x0, Simul_time, nmpc, ref);

% Plot pose
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X_50, U_50, Ref_50);
ph.fig.Name = 'NMPC_50'; % Set a figure title

%% Comparison
figure
plot(T,U_15(4,:),T,U_50(4,:))
legend('15°', '50°')
ylabel('Pdiff (%)')