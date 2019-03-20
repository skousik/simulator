classdef planner < handle

properties
% generic planner properties
    name
    bounds % world bounds plus planner-specific buffer
    default_buffer % minimum amount to buffer obstacles, given by the world
    HLP % high level planner
    current_plan ;
    current_obstacles ;
    verbose = 0 ;
    timeout = 1 ; % time allowed for "replan" function to execute
    t_plan = 1 ; % same as timeout; just for notational purposes
    t_move = 0.1 ;% amount of time the planner expects the agent to move
    info = [] ; % information structure to keep a log when planning
    plot_data % data for current plot
    plot_waypoints_flag = false ;
end

methods
%% constructor
    function P = planner(timeout,verbose_level)
        if nargin > 0
            P.timeout = timeout ;
            P.t_plan = timeout ;
        end

        if nargin > 1
            P.verbose = verbose_level ;
        end
    end

%% setup
    function setup(~,~,~)
    % P.setup(agent_info,world_info)
    %
    % ADD COMMENTS
        warning('Planner setup function is undefined!')
    end

%% replan
    function [T_nom,U_nom,Z_nom] = replan(~,~,~)
    % [T_nom,U_nom,Z_nom] = P.replan(A,O)
    %
    % ADD COMMENTS
        warning('Planner replan function is undefined!')

        if isempty(P.HLP)
            warning('The high level planner is also undefined!')
        end

        T_nom = [] ; U_nom = [] ; Z_nom = [] ;
    end
    
%% plotting
    function plot(P,color)
        if nargin < 2
            color = [0 0 1] ;
        end
        
        % check if a figure is up; if so, create a new figure,
        % otherwise update the existing data
        fh = get(groot,'CurrentFigure') ;

        if isempty(A.plot_data) || isempty(fh)
            hold on 
            
            % plot anticipated trajectory from planner
            if ~isempty(P.current_plan)
                P.plot_data.current_plan = plot(P.current_plan(1,:),P.current_plan(2,:),...
                                                ':','LineWidth',2,'Color',color) ;
            end

            % plot waypoints
            if P.plot_waypoints_flag
                error('Plotting waypoints is not implemented yet!')
                % P.HLP.plotWaypoints() ;
            end
            
            hold off
        else
            P.plot_data.current_plan.XData = P.current_plan(1,:) ;
            P.plot_data.current_plan.YData = P.current_plan(2,:) ;
        end
    end
    
%% utility
    function vdisp(P,s,l)
    % Display a string s if the message's verbose level l is greater
    % than or equal to the planner's verbose level.
        if nargin < 3
            l = 1 ;
        end
        if P.verbose >= l
            if ischar(s)
                disp(['    P: ',s])
            else
                disp('    P: String not provided!')
            end
        end
    end
end
end