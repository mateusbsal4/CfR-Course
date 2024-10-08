% Visualize2
%
% Description: This function calls the 'Visualize' function that is
%              provided in the original package. This function makes a few
%              updates to the subplots' title, axis label, and plot legend.
%              Note: Make changes as needed.
%
% Inputs:
%       Sim_Out: A struct containing simulation outputs generated by
%                Quad_Simulator().
%       Model_Param: A vector containing quadrotor model parameters.
%
% Output:
%       fig_hdl: Plot handle
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
% This script is adapted from the course on Optimal & Learning Control for
% Autonomous Robots at the Swiss Federal Institute of Technology in Zurich
% (ETH Zurich). Course Instructor: Jonas Buchli. Course Webpage:
% http://www.adrlab.org/doku.php/adrl:education:lecture:fs2015
%
% --
% Revision history
% [20.02.01, SZ]    first version

function [fig_hdl] = Visualize2(Sim_Out, Model_Param)
    % Provided visualization function
    Visualize(Sim_Out, Model_Param, 'plot_mode', 1);
    
    % A few updates to the visualization plots
    fig_hdl = gcf;
    h = get(fig_hdl, 'children');
 
    % Ylabels of subplots
    %h(3).YLabel.String = 'rotor-speed [(rad/s)^2]';
    h(3).YLabel.String = 'rotor-thrust [N]';
    h(3).YLabel.HorizontalAlignment = 'center';
    y_label_loc = min([h(1).YLabel.Position(1), ...
        h(2).YLabel.Position(1), h(3).YLabel.Position(1)]);
    h(1).YLabel.Position(1) = y_label_loc;
    h(2).YLabel.Position(1) = y_label_loc;
    h(3).YLabel.Position(1) = y_label_loc;
    
    % Add legends to plots
    temp = legend(h(1), 'vx', 'vy', 'vz', 'location', 'nw', ...
        'orientation', 'horizontal'); temp.FontSize = 8;
	temp = legend(h(2), 'x', 'y', 'z', 'location', 'nw', ...
        'orientation', 'horizontal'); temp.FontSize = 8;

    % Ylims
    h(1).YLim(2) = diff(h(1).YLim)*0.23 + h(1).YLim(2);
    h(2).YLim(2) = diff(h(2).YLim)*0.23 + h(2).YLim(2);
    
    % Remove subplot titles
    h(1).Title.String = '';
    h(2).Title.String = '';
    
    % Axis labels of the animation plot
    set(h(4), 'position', [0.1 0.11 0.4 0.815], 'units', 'normalized');
    h(4).XLabel.String = 'x [m]';
    h(4).YLabel.String = 'y [m]';
    h(4).ZLabel.String = 'z [m]';
    
    % Maximize window
	fig_hdl.WindowState = 'maximized';
end