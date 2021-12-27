Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim() % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us)

[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us)