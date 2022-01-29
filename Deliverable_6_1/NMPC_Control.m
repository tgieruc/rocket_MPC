function opti_eval = NMPC_Control(rocket, H)

import casadi.*
opti = casadi.Opti(); % Optimization problem

N = ceil(H/rocket.Ts); % MPC horizon
nx = 12; % Number of states
nu = 4;  % Number of inputs

% Decision variables (symbolic)
X_sym = opti.variable(nx, N); % state trajectory
U_sym = opti.variable(nu, N-1);   % control trajectory)

% Parameters (symbolic)
x0_sym  = opti.parameter(nx, 1);  % initial state
ref_sym = opti.parameter(4, 1);   % target position

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
M = zeros(nx, nu);
M(10:12,1:3) = eye(3); %x y z 
M(6, 4) = 1; %gamma
%         wx wy wz a b g   vx vy vz x    y    z
Q = diag([50 50 1  1 1 500 1  1  1  5000 5000 5000]);
R = diag([0.0001 0.0001 1 0.0001]);

f_discrete = @(x,u) RK4(x,u,1/10,rocket);

obj=0;

for i = 1 : N-1
    opti.subject_to(X_sym(:, i+1) == f_discrete(X_sym(:,i), U_sym(:,i)))
    obj = obj + (X_sym(:, i) - M * ref_sym)'*Q*(X_sym(:, i) - M * ref_sym) + U_sym(:, i)'*R*U_sym(:, i);
end

opti.subject_to(X_sym(:, 1) == x0_sym);
opti.subject_to(-deg2rad(85) <= X_sym(5,:) <= deg2rad(85))

opti.subject_to([-0.26; -0.26; 50; -20] <= U_sym <= [0.26; 0.26; 80; 20])

opti.minimize(obj + (X_sym(:, N) - M * ref_sym)'*Q*(X_sym(:,N) - M * ref_sym));




% YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ---- Setup solver ------
ops = struct('ipopt', struct('print_level', 0, 'tol', 1e-3), 'print_time', false);
opti.solver('ipopt', ops);

% Create function to solve and evaluate opti
opti_eval = @(x0_, ref_) solve(x0_, ref_, opti, x0_sym, ref_sym, U_sym);
end

function u = solve(x0, ref, opti, x0_sym, ref_sym, U_sym)

% ---- Set the initial state and reference ----
opti.set_value(x0_sym, x0);
opti.set_value(ref_sym, ref);

% ---- Solve the optimization problem ----
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');

u = opti.value(U_sym(:,1));

% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
opti.set_initial(opti.lam_g, sol.value(opti.lam_g));
end

function [x_next] = RK4(X,U,h,rocket)
%
% Inputs : 
%    X, U current state and input
%    h    sample period
%    f    continuous time dynamics f(x,u)
% Returns
%    State h seconds in the future
%

% Runge-Kutta 4 integration
% write your function here
   k1 = rocket.f(X, U);
   k2 = rocket.f(X+h/2*k1, U);
   k3 = rocket.f(X+h/2*k2, U);
   k4 = rocket.f(X+h*k3,   U);
   x_next = X + h/6*(k1+2*k2+2*k3+k4);
end
