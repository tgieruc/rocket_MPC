addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20;
rocket = Rocket(Ts);
H = 4;
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);

[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

mpc_x = MPC_Control_x(sys_x, Ts, H);
mpc_y = MPC_Control_y(sys_y, Ts, H);
mpc_z = MPC_Control_z(sys_z, Ts, H);
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);

% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

rocket.mass = 1.783; % Manipulate mass for simulation

%% Without Estimator
Tf = 30;
ref = @(t_, x_) rocket.MPC_ref(t_, Tf);
x0 = zeros(12,1);
[T, X_without, U_without, Ref] = rocket.simulate_f(x0, Tf, mpc, ref);
% Plot pose
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X_without, U_without, Ref);
ph.fig.Name = 'Without estimator'; % Set a figure title


%% With Estimator
Tf = 30;
ref = @(t_, x_) rocket.MPC_ref(t_, Tf);
x0 = zeros(12,1);
[T, X_with, U_with, Ref, Zhat] = rocket.simulate_f_est_z(x0, Tf, mpc, ref, mpc_z, sys_z);

% Plot pose
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X_with, U_with, Ref);
ph.fig.Name = 'With estimator'; % Set a figure title


%% Comparison
figure
plot(T,U_with,T,U_without)
plot(T,U_with(3,:),T,U_without(3,:))
legend('with estimator', 'without estimator')
ylabel('P_{avg} (%)')