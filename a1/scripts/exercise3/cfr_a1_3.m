% cfr_a1_3: Main script for Problem 1.3 Approximate Dynamic Programming.
%
% adapted from: Borrelli, Francesco: "ME 231 Experiential Advanced Control
% Design I"
%
% --
% Control for Robotics
% Summer 2023
% Assignment 1
%
% --
% Technical University of Munich
% Learning Systems and Robotics Lab
%
% Course Instructor:
% Angela Schoellig
% schoellig@utias.utoronto.ca
%
% Teaching Assistants: 
% SiQi Zhou: siqi.zhou@tum.de
% Lukas Brunke: lukas.brunke@tum.de
%
% --
% Revision history
% [22.01.17, LB]    first version
% [22.01.23, LB]    added 2 (c) to the code, removed N
%
% --
% Revision history
% [22.01.17, LB]    first version
% [22.01.24, LB]    updated horizon and initial state

clear all
close all
clc

%% set up system

% inverted pendulum parameters
l = 1.0; % length
g = 9.81; % gravitational constant
m = 1.0; % mass

% create inverted pendulum system
sys = InvertedPendulum(l, g, m);

% controller parameters
Q = diag([100, 10]);
R = 0.01;

N = 25;

% linearization point
x_up = [pi; 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: YOUR CODE HERE - Exercise 3 (a)
A_c = [0 1; g/l 0]
B_c = [0; 1/m*l^2]
delta_t = 0.1;
sysc = ss(A_c, B_c, [], []);
sysd = c2d(sysc, delta_t);

A_d = sysd.A
B_d = sysd.B



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% cost functions

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: YOUR CODE HERE - Exercise 3 (b)
stage_cost = @(x_k, u_k) x_k'*Q*x_k + R*u_k^2; 
J_n = @(x_n)  x_n'*Q*x_n;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% calculate optimal control using dynamic programming and gridding

% grid state-space
num_points_x1 = 10;
num_points_x2 = 5;
X1 = linspace(-pi/4, pi/4, num_points_x1)
X2 = linspace(-pi/2, pi/2, num_points_x2)

% allocate arrays for optimal control inputs and cost-to-go 
U = zeros(num_points_x1, num_points_x2);
J = zeros(num_points_x1, num_points_x2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: YOUR CODE HERE - Exercise 3 (c)
%initialization
for i = 1:length(X1)
    for j = 1:length(X2)
        x = [X1(i);X2(j)];
        J(i, j) = J_n(x);
    end
end     
%recursion


x_next_1 = @(x, u) min(max(A_d(1,:)*x + B_d(1)*u, min(X1)), max(X1));       %this is required to bound the values of the linearized position and velocities to their respective
x_next_2 = @(x, u) min(max(A_d(2,:)*x + B_d(2)*u, min(X2)), max(X2));       %maximum or minimum values if the control input causes the state values to exceed these boundaries 

for k=24:-1:0
    for i = 1:length(X1)
        for j = 1:length(X2)
            x = [X1(i) ; X2(j)]
            cost = @(u) stage_cost(x, u) + interp2(X1, X2, J', x_next_1(x, u), x_next_2(x,u), 'spline');
            U(i,j) = fminunc(cost, U(i,j)); 
            J(i, j) = cost(U(i,j));

        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% plot optimal control and cost-to-go
figure
subplot(1, 2, 1)
surf(X1, X2, U')
xlabel('x_1')
ylabel('x_2')
zlabel('u')
title("Parameters Q = diag(1, 0.1), R = 1")
subplot(1, 2, 2)
surf(X1, X2, J')
xlabel('x_1')
ylabel('x_2')
zlabel('J')
title("Parameters Q = diag(1, 0.1), R = 1")

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Exercise 3(d)4
%When r is increased, more weight is given to the input effort in, so the values of the policy decrease in magnitude.
%On the other hand, an increase of the (eigenvalues) of the weighting matrix Q leads to an increase of the input effort while the deviation of the is smaller (from initially pi/6 or 2.61 radians to 2.07) 
% Naturally, increasing any of these values leads to an increase of the final cost
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%% apply control law and simulate inverted pendulum
% create the controlled inverted pendulum system
control_sys = InvertedPendulum(l, g, m, X1, X2, U, x_up);

% initial condition
x0 = x_up + [-pi/6; 0];

% duration of simulation
t = [0, 10];

% simulate control system
[t, x] = ode45(@control_sys.controlled_dynamics, t, x0);

% determine control inputs from trajectory
u = zeros(size(t));
for i = 1 : length(t)
    u(i) = control_sys.mu(x(i, :)' - x_up);
end

%% plot state and input trajectories
figure
subplot(2, 1, 1)
hold on
plot(t, x(:, 1))
plot(t, x(:, 2))
plot(t, pi*ones(length(t)));
xlabel('t')
ylabel('x_1 and x_2')
title("Parameters Q = diag(1, 0.1), R = 1")
hold off
legend('\theta','d\theta/dt')
grid on
subplot(2, 1, 2)
plot(t, u)
xlabel('t')
ylabel('u')
title("Parameters Q = diag(1, 0.1), R = 1")
grid on



%{
 

%cfr_a1_3: Main script for Problem 1.3 Approximate Dynamic Programming.
%
% adapted from: Borrelli, Francesco: "ME 231 Experiential Advanced Control
% Design I"
%
% --
% Control for Robotics
% Summer 2023
% Assignment 1
%
% --
% Technical University of Munich
% Learning Systems and Robotics Lab
%
% Course Instructor:
% Angela Schoellig
% schoellig@utias.utoronto.ca
%
% Teaching Assistants: 
% SiQi Zhou: siqi.zhou@tum.de
% Lukas Brunke: lukas.brunke@tum.de
%
% --
% Revision history
% [22.01.17, LB]    first version
% [22.01.23, LB]    added 2 (c) to the code, removed N
%
% --
% Revision history
% [22.01.17, LB]    first version
% [22.01.24, LB]    updated horizon and initial state

clear all
close all
clc

%% set up system

% inverted pendulum parameters
l = 1.0; % length
g = 9.81; % gravitational constant
m = 1.0; % mass

% create inverted pendulum system
sys = InvertedPendulum(l, g, m);

% controller parameters
Q = diag([1, 0.1]);
R = 1;
N = 25;

% linearization point
x_up = [pi; 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: YOUR CODE HERE - Exercise 3 (a)
A_c = [0 1; g/l 0];
B_c = [0; 1/m*l^2];
delta_t = 0.1;
sysc = ss(A_c, B_c, zeros(2), zeros(size(B_c)));
sysd = c2d(sysc, delta_t);

A_d = sysd.A
B_d = sysd.B




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% cost functions

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: YOUR CODE HERE - Exercise 3 (b)
stage_cost = @(x_k, u_k) x_k'*Q*x_k + R*u_k^2; 
J_n = @(x_n)  x_n'*Q*x_n;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% calculate optimal control using dynamic programming and gridding

% grid state-space
num_points_x1 = 10;
num_points_x2 = 5;
X1 = linspace(-pi/4, pi/4, num_points_x1);
X2 = linspace(-pi/2, pi/2, num_points_x2);

% allocate arrays for optimal control inputs and cost-to-go 
U = zeros(num_points_x1, num_points_x2);
J = zeros(num_points_x1, num_points_x2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: YOUR CODE HERE - Exercise 3 (c)
%initialization
for i = 1:length(X1)
    for j = 1:length(X2)
        x = [X1(i);X2(j)];
        J(i, j) = J_n(x);
    end
end     
%recursion
J

for k=25:-1:0
    for i = 1:length(X1)
        for j = 1:length(X2)
            x = [X1(i) ; X2(j)];
            min_cost = inf;
            for u = -20:0.1:20  % Adjust this range based on your control input limits
                x_next = A_d * x + B_d * u;
                if x_next(1)>pi/4 || x_next(1) < -pi/4 || x_next(2) < -pi/2 || x_next(2) > pi/2
                    continue;
                end
                cost = stage_cost(x, u) + interp2(X1, X2, J', x_next(1), x_next(2));
                if cost < min_cost
                    min_cost = cost;
                    U(i, j) = u;
                end
            end
            J(i, j) = min_cost;

        end
    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% plot optimal control and cost-to-go
figure
subplot(1, 2, 1)
surf(X1, X2, U')
xlabel('x_1')
ylabel('x_2')
zlabel('u')
subplot(1, 2, 2)
surf(X1, X2, J')
xlabel('x_1')
ylabel('x_2')
zlabel('J')




%% apply control law and simulate inverted pendulum
% create the controlled inverted pendulum system
control_sys = InvertedPendulum(l, g, m, X1, X2, U, x_up);

% initial condition
x0 = x_up + [-pi/6; 0];

% duration of simulation
t = [0, 10];

% simulate control system
[t, x] = ode45(@control_sys.controlled_dynamics, t, x0);

% determine control inputs from trajectory
u = zeros(size(t));
for i = 1 : length(t)
    u(i) = control_sys.mu(x(i, :)' - x_up);
end

%% plot state and input trajectories
figure
subplot(2, 1, 1)
hold on
plot(t, x(:, 1))
plot(t, x(:, 2))
xlabel('t')
ylabel('x_1 and x_2')
hold off
legend('\theta','d\theta/dt')
grid on
subplot(2, 1, 2)
plot(t, u)
xlabel('t')
ylabel('u')
grid on

%}