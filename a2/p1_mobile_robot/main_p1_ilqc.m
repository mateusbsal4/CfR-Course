% main_p1_ilqc: Main script for Problem 2.1 ILQC controller design.
%
% --
% Control for Robotics
% Assignment 2
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
% [20.01.31, SZ]    first version

clear all;
close all;

%% General
% add subdirectories
addpath(genpath(pwd));

% add task
task_ilqc = task_design();
N = length(task_ilqc.start_time:task_ilqc.dt:task_ilqc.end_time);

% add model
const_vel = 1; % assume constant forward speed
model = generate_model(const_vel);

% save directory
save_dir = './results/';

% initialize controller
load(strcat(save_dir, 'lqr_controller'));
controller_ilqc = controller_lqr;

% flags
plot_on = true;
save_on = true;


function [Ak, Bk] = mobile_robot_lin(x_bar_k, u_bar_k, delta_t, v)
    h_bar = x_bar_k(2);
    Ak = [1 delta_t*v*cos(h_bar); 0 1];
    Bk = [0 ;delta_t];
end 

function [g_barN, q_N, Q_N] = terminal_cost_quad(Q_t, x_goal, x_N_bar)
    g_barN = 0.5*(x_N_bar - x_goal)'*Q_t*(x_N_bar - x_goal);
    q_N = Q_t*(x_N_bar - x_goal);
    Q_N = Q_t;
end

function [g_k_bar, q_k, Q_k, r_k, R_k, P_k] = stage_cost_quad(Q_s, R_s, x_goal, delta_t, x_k_bar, u_k_bar)
    g_k_bar = 0.5*(x_k_bar - x_goal)'*Q_s*(x_k_bar - x_goal) + 0.5*R_s*u_k_bar^2;
    q_k = Q_s*(x_k_bar - x_goal);
    Q_k = Q_s;
    r_k = R_s*u_k_bar;
    R_k = R_s;
    P_k = 0;
end

function [theta_kff, theta_kfb, s_k_bar, s_k, S_k] = update_policy(Ak, Bk, g_k_bar, q_k, Q_k, r_k, R_k, P_k, sk_barnext, s_k_next, S_k_next, x_k_bar, u_k_bar)
    l_k = r_k + Bk'*s_k_next;
    G_k = P_k + Bk'*S_k_next*Ak;
    H_k = R_k + Bk'*S_k_next*Bk;
    K_k  = -G_k/H_k;
    delta_ukff = -l_k/H_k;
    theta_kff = u_k_bar +delta_ukff + -K_k*x_k_bar;
    theta_kfb = -G_k/H_k;
    s_k_bar = g_k_bar + sk_barnext +l_k*delta_ukff + (H_k*delta_ukff^2)/2;
    s_k = q_k + Ak'*s_k_next + K_k'*H_k*delta_ukff + K_k'*l_k + G_k'*delta_ukff;
    S_k = Q_k + Ak'*S_k_next*Ak +K_k'*H_k*K_k + G_k'*K_k + K_k'*G_k;

end


%% [Problem 2.1 (j)] Iterative Linear Quadratic Controller
% =========================== [TODO] ILQC Design ==========================
% Design an ILQC controller based on the linearized dynamics and
% quadratized costs. The cost function of the problem is specified in
% 'task_ilqc.cost' via the method 'task_design()'.
 % time
dt = task_ilqc.dt;
% simulation parameters
v = const_vel;
num_it = task_ilqc.max_iteration;
x_goal = task_ilqc.goal_x;
% state and input dimensions
state_dim = 2;
input_dim = 1;

% initializations linearized states
x_bars = zeros(state_dim, N);
u_bars = zeros(input_dim, N-1);

% cost Q and R
Q_s = task_ilqc.cost.params.Q_s;
R_s = task_ilqc.cost.params.R_s;
Q_t = task_ilqc.cost.params.Q_t;

x_bars(:,1) = task_ilqc.start_x;
for l = 0:1:num_it
    %%Forward pass %
    cur_x_bar = x_bars(:, 1);
    for k = 1:1:N-1
        cur_u_bar = controller_ilqc(:,k)'*[1; cur_x_bar];
        cur_x_bar = cur_x_bar + dt*model.f(cur_x_bar, cur_u_bar, model);  
        x_bars(:,k+1) = cur_x_bar;
        u_bars(:,k) = cur_u_bar;
    end
    %%%%
    %Backward pass initialization%
    x_N_bar = cur_x_bar;
    [s_bar, s, S] = terminal_cost_quad(Q_t, x_goal, x_N_bar);        %cost to go at time N
    %%%%
    for k = N-1:-1:1
        x_k_bar = x_bars(:, k);
        u_k_bar = u_bars(:, k);
        [Ak, Bk] = mobile_robot_lin(x_k_bar, u_k_bar, dt, v);
        [g_k_bar, q_k, Q_k, r_k, R_k, P_k] = stage_cost_quad(Q_s, R_s, x_goal, dt, x_k_bar, u_k_bar);
        [theta_kff, theta_kfb, s_bar, s, S] = update_policy(Ak, Bk, g_k_bar, q_k, Q_k, r_k, R_k, P_k, s_bar, s, S, x_k_bar, u_k_bar);
        controller_ilqc(1, k) = theta_kff;
        controller_ilqc(2, k) = theta_kfb(1);
        controller_ilqc(3, k) = theta_kfb(2);
    end

end
% =========================================================================

%% Simulation
sim_out_ilqc = mobile_robot_sim(model, task_ilqc, controller_ilqc);
%sim_out_ilqc = mobile_robot_sim_test(model, task_ilqc, controller_ilqc, 0.5);
fprintf('\n\ntarget state [%.3f; %.3f]\n', task_ilqc.goal_x);
fprintf('reached state [%.3f; %.3f]\n', sim_out_ilqc.x(:,end));

%% Plots
if plot_on
    plot_results(sim_out_ilqc);
end

%% Save controller and simulation results
if save_on
    if ~exist(save_dir, 'dir')
       mkdir(save_dir); 
    end
    
    % save controller and simulation results
	save(strcat(save_dir, 'ilqc_controller'), 'controller_ilqc', ...
        'sim_out_ilqc', 'task_ilqc'); 
end