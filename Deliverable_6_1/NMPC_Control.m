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

%% System description

%call the dynamic of the system 
% Fly along the x/y/z axis. (from part1.m)
d1 = 0;
d2 = 0.008;
Pavg = 80;
Pdiff = 0;
u = [d1, d2, Pavg, Pdiff]'; % (Assign appropriately)

w = [0, 0, 0];
phi = [0, pi/2, 0];
v = [0, 0, 0];
p = [0, 0, 0];
x = [w, phi, v, p]'; % (Assign appropriately)

f_discrete = @(x,u) RK4(x,u,N,rocket);

%constraints:
%abs(beta) < 85° = 1,48353 rad --> soft constraints ? 
% abs(delta1) and abs(delta2) < 15° = 0.26 rad 
% 0.2 < Pavg < 0.8 
%others constraints were only there because of the linearization --> not
%useful now


% opti.minimize(...
%   -10*sum(X_sym(2,:))  + ... % Max velocity
%   0.1*U_sym(1,:)*U_sym(1,:)' + ... % Minimize accel
%   10*U_sym(2,:)*U_sym(2,:)'); % Minimize braking

opti.minimize(...
  -10*sum(X_sym(2,:))  + ... % Max velocity
  0.1*U_sym(1,:)*U_sym(1,:)'); % Minimize accel

% ---- multiple shooting --------
for k=1:N % loop over control intervals

  opti.subject_to(X_sym(:,k+1) == f_discrete(X_sym(:,k), U_sym(:,k)));
 
end

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




function [x_next] = RK4(X,U,h,rocket.f)
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
