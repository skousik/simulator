classdef planner < handle

properties
% generic planner properties
    name
    bounds % world bounds plus planner-specific buffer
    default_buffer % minimum amount to buffer obstacles, given by the world
    HLP % high level planner
    current_plan
    current_obstacles
    verbose
    timeout % time allowed for "replan" function to execute
    t_plan % same as timeout; just for notational purposes
    t_move % amount of time the planner expects the agent to move
    info % information structure to keep a log when planning
    plot_waypoints_flag
end

methods
%% constructor
    function P = planner(timeout,verbose_level)
        if nargin > 0
            P.timeout = timeout ;
            P.t_plan = timeout ;
        else
            P.timeout = 1 ;
            P.t_plan = 1 ;
        end

        if nargin > 1
            P.verbose = verbose_level ;
        else
            P.verbose = 0 ;
        end

        P.HLP = [] ;
        P.current_plan ;
        P.info = [] ;
        P.plot_waypoints_flag = false ;
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
    function plot(P)
        P.plotInLoop(1,[1 0 0]) ;    
    end
    
    function plotInLoop(P,n,c)
        P.vdisp('Plotting in loop',4)

        if nargin < 3
            c = [1 0 0] ;
        end

        figure(n) ;

        % plot anticipated trajectory from planner
        if ~isempty(P.current_plan)
            plot(P.xy_plan(1,:),P.xy_plan(2,:),':','LineWidth',2,'Color',c)
        end
        
        % plot waypoints
        if P.plot_waypoints_flag
            P.HLP.plotWaypoints()
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