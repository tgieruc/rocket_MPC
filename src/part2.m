clc;
close all;

Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim() % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us) % Linearize the nonlinear model about trim point

[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us)

syms wx wy wz alpha beta gamma vx vy vz x z y d1 d2 Pavg Pdiff
x_vec = [ wx wy wz alpha beta gamma vx vy vz x z y]';
u_vec = [d1 d2 Pavg Pdiff]';
