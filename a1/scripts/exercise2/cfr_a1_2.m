% cfr_a1_2: Main script for Problem 1.2 Dynamic Programming for a 
%               Robot Vacuum Cleaner.
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

clear all
close all
clc

%% calculate optimal control using dynamic programming

% initialize the grid world
grid = GridWorld();

% allocate arrays for optimal control inputs and cost-to-go 
%U = zeros(grid.num_rows, grid.num_columns);
%J = zeros(grid.num_rows, grid.num_columns);

% set the cost for the obstacle
%J(grid.obstacle_pos(1), grid.obstacle_pos(2)) = inf;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: YOUR CODE HERE - Exercise 2 (a)
function [J, U] = value_iteration(grid)
    J = inf(grid.num_rows, grid.num_columns);
    J(grid.charger_pos(1), grid.charger_pos(2)) = 0;

    max_iter = 500;
    tolerance = 1e-3;
    iter = 0;
    while iter< max_iter
        prev_J = J;
        for i = 1:grid.num_rows     
            for j = 1:grid.num_columns
                if ~(i == grid.charger_pos(1) && j == grid.charger_pos(2))          %excluding the obstacle state
                    min_cost = inf;
                    x = [i;j];           %current state k
                    for a = grid.available_actions(x)           %finding the costs to go for each possible action 
                        x_next = grid.next_state(x, a);
                        action_cost = grid.stage_cost(x_next(1), x_next(2)) + J(x_next(1), x_next(2));  
                        min_cost = min(min_cost, action_cost);
                    end
                    J(i, j) = min_cost;
                end
            end
        end
        % Check for convergence
        if max(abs(prev_J - J)./prev_J) < tolerance
            break;
        end
        iter = iter + 1;
    end

    U = zeros(grid.num_rows, grid.num_columns);
    % Extract optimal policy once the costs to go are correct

    for i = 1:grid.num_rows
        for j = 1:grid.num_columns
            if ~(i == grid.charger_pos(1) && j == grid.charger_pos(2)) 
                x = [i;j];
                min_value = inf;
                best_action = 0;
                for a = grid.available_actions(x)
                    x_next = grid.next_state(x, a);
                    action_cost = grid.stage_cost(x_next(1), x_next(2)) + J(x_next(1), x_next(2));
                    if action_cost < min_value
                        min_value = action_cost;
                        best_action = a;
                    end
                end
                U(i, j) = best_action;
            end
        end
    end
end

[J, U] = value_iteration(grid);
disp('Cost-to-go:');
disp(J);
disp('Optimal Policy:');
disp(U);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Simulate robot vacuum cleaner
x_0 = [4; 3];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: YOUR CODE HERE - Exercise 2 (b)

function optimal_actions = best_path(grid, x_0)
    [~, U] = value_iteration(grid);
    x = x_0;
    optimal_actions = [];
    while ~(isequal(x, grid.charger_pos))
        opt_action = U(x(1), x(2));
        x = grid.next_state(x, opt_action);
        optimal_actions = [optimal_actions, opt_action];

    end
end

optimal_actions = best_path(grid, x_0)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

grid.plot_moves(x_0, optimal_actions)

%% Simulate robot vacuum cleaner
x_0 = [4; 3];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: YOUR CODE HERE - Exercise 2 (c)  
grid.cost_dirt = 5;
grid.stage_cost(2, 1) = grid.cost_dirt;
[J,U] = value_iteration(grid);
optimal_actions = best_path(grid, x_0)
grid.plot_moves(x_0, optimal_actions)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Exercise 2 (d)
% One solution would be to use a time variant cost for the charging station. Initally setting the cost-to-go of the 
% charger to infinity and changing it to zero after a certain number of timesteps k
% would force the robot to leave the charging station
%
%
%