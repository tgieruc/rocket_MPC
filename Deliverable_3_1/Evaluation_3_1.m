Ts = 1/20; % Sample time
rocket = Rocket(Ts);

[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys x, sys y, sys z, sys roll] = rocket.decompose(sys, xs, us);

% Design MPC controller
H = ; % Horizon length in seconds %To be completed -> on met quoi ?

mpc_x = MPC_Control_x(sys_x, Ts, H);
% Get control input
ux = mpc_x.get_u(x); %--> Pas compris à quoi ns sert ce ux ? C'est notre control input 


[T, X_sub, U_sub] = rocket.simulate(sys_x, x0, Tf, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);