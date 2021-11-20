% simulate NL system with given parameters
% inputs:
% f, g: function handles for f(x,u) and g(x,u)
% u: input as a function handle of time u(t)
% t: either [t0, tf] with auto step or a row vector of time points to use
% x0: initial state
function [t_nl, u_nl, x_nl, y_nl] = simNL(f, g, u, t, x0)
    opts = odeset('RelTol',1e-4, 'Refine', 4); % solver settings
    %[t_nl, x_nl] = ode45(@(t,x) f(x,u(t)),t,x0); % non-stiff solver
    [t_nl, x_nl] = ode15s(@(t,x) f(x,u(t)),t,x0, opts); % stiff solver
    %[t_nl, x_nl] = ode23s(@(t,x) f(x,u(t)),t,x0); % stiff solver
    t_nl = t_nl'; % % t(1,i) is time at time i
    x_nl = x_nl'; % x(i,j) is state i at time j
    u_nl = u(t_nl); % u as a vector vs time
    y_nl = g(x_nl, u_nl); % y(i,j) is output i at time j
end