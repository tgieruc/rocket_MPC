close all;

Ts = 1/20;
rocket = Rocket(Ts);

%% Ascend vertically without tipping over.
d1 = 0;
d2 = 0;
Pavg = 80;
Pdiff = 0;
u = [d1, d2, Pavg, Pdiff]'; % (Assign appropriately)
[b_F, b_M] = rocket.getForceAndMomentFromThrust(u)
w = [0, 0, 0];
phi = [0, 0, 0];
v = [0, 0, 0];
p = [0, 0, 0];
x = [w, phi, v, p]'; % (Assign appropriately)
Tf = 2.0; % Time to simulate for
x0 = x;
[T, X, U] = rocket.simulate_f(x0, Tf, u); % Simulate nonlinear dynamics f
rocket.anim_rate = 1.0;
rocket.vis(T, X, U);

%% Descend vertically without tipping over.
d1 = 0;
d2 = 0;
Pavg = 30;
Pdiff = 0;
u = [d1, d2, Pavg, Pdiff]'; % (Assign appropriately)
[b_F, b_M] = rocket.getForceAndMomentFromThrust(u)
w = [0, 0, 0];
phi = [0, 0, 0];
v = [0, 0, 0];
p = [0, 0, 0];
x = [w, phi, v, p]'; % (Assign appropriately)
Tf = 2.0; % Time to simulate for
x0 = x;
[T, X, U] = rocket.simulate_f(x0, Tf, u); % Simulate nonlinear dynamics f
rocket.anim_rate = 1.0;
rocket.vis(T, X, U);

%% Rotate about its body z axis.
d1 = 0;
d2 = 0;
Pavg = 80;
Pdiff = 15;
u = [d1, d2, Pavg, Pdiff]'; % (Assign appropriately)
[b_F, b_M] = rocket.getForceAndMomentFromThrust(u)
w = [0, 0, 0];
phi = [0, 0, 0];
v = [0, 0, 0];
p = [0, 0, 0];
x = [w, phi, v, p]'; % (Assign appropriately)
Tf = 2.0; % Time to simulate for
x0 = x;
[T, X, U] = rocket.simulate_f(x0, Tf, u); % Simulate nonlinear dynamics f
rocket.anim_rate = 1.0;
rocket.vis(T, X, U);

%% Rotate about its body y axis.
d1 = 0.01;
d2 = 0.01;
Pavg = 80;
Pdiff = 0;
u = [d1, d2, Pavg, Pdiff]'; % (Assign appropriately)
[b_F, b_M] = rocket.getForceAndMomentFromThrust(u)
w = [0, 0, 0];
phi = [0, 0, 0];
v = [0, 0, 0];
p = [0, 0, 0];
x = [w, phi, v, p]'; % (Assign appropriately)
Tf = 2.0; % Time to simulate for
x0 = x;
[T, X, U] = rocket.simulate_f(x0, Tf, u); % Simulate nonlinear dynamics f
rocket.anim_rate = 1.0;
rocket.vis(T, X, U);

%% Rotate about its body x axis.
d1 = 0.01;
d2 = 0;
Pavg = 80;
Pdiff = 0;
u = [d1, d2, Pavg, Pdiff]'; % (Assign appropriately)
[b_F, b_M] = rocket.getForceAndMomentFromThrust(u)
w = [0, 0, 0];
phi = [0, 0, 0];
v = [0, 0, 0];
p = [0, 0, 0];
x = [w, phi, v, p]'; % (Assign appropriately)
Tf = 2.0; % Time to simulate for
x0 = x;
[T, X, U] = rocket.simulate_f(x0, Tf, u); % Simulate nonlinear dynamics f
rocket.anim_rate = 1.0;
rocket.vis(T, X, U);

%% Fly along the x/y/z axis.
d1 = 0;
d2 = 0.008;
Pavg = 80;
Pdiff = 0;
u = [d1, d2, Pavg, Pdiff]'; % (Assign appropriately)
[b_F, b_M] = rocket.getForceAndMomentFromThrust(u)
w = [0, 0, 0];
phi = [0, pi/2, 0];
v = [0, 0, 0];
p = [0, 0, 0];
x = [w, phi, v, p]'; % (Assign appropriately)
Tf = 2.0; % Time to simulate for
x0 = x;
[T, X, U] = rocket.simulate_f(x0, Tf, u); % Simulate nonlinear dynamics f
rocket.anim_rate = 1.0;
rocket.vis(T, X, U);

%% Hover in space.
d1 = 0;
d2 = 0;
Pavg = 56.666;  
Pdiff = 0;
u = [d1, d2, Pavg, Pdiff]'; % (Assign appropriately)
[b_F, b_M] = rocket.getForceAndMomentFromThrust(u)
w = [0.2, 0, 0];
phi = [0, 0, 0];
v = [0, 0, 0];
p = [0, 0, 0];
x = [w, phi, v, p]'; % (Assign appropriately)
Tf = 2.0; % Time to simulate for
x0 = x;
[T, X, U] = rocket.simulate_f(x0, Tf, u); % Simulate nonlinear dynamics f
rocket.anim_rate = 1.0;
rocket.vis(T, X, U);