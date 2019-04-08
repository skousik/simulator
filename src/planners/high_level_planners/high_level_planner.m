classdef high_level_planner < handle
% Class: high_level_planner
%
% For use with simulator. The high-level plannee is responsible for
% creating coarse plans for an agent to navigate a world. This class is
% mean to be called by a planner object in its replan method while planning
% online; it is not called directly by simulator. It turns out that having
% a coarse planner that generates, e.g., waypoints, is really useful for
% enabling a mid-level trajectory planner to do its thing.

%% properties
properties
    default_lookahead_distance
    waypoint_reached_radius
    waypoints
    N_waypoints
    current_waypoint
    current_waypoint_index
    waypoints_include_heading
    verbose
    plot_data
end

methods
%% constructor
    function HLP = high_level_planner(verbose_level)
        if nargin < 1
            verbose_level = 0 ;
        end
        
        HLP.default_lookahead_distance = 1 ;
        HLP.waypoints = [] ;
        HLP.N_waypoints = 0 ;
        HLP.waypoint_reached_radius = 1 ;
        HLP.waypoints_include_heading = false ;
        HLP.current_waypoint = [] ;
        HLP.current_waypoint_index = [] ;
        HLP.verbose = verbose_level ;
        HLP.plot_data.waypoints = [] ;
        HLP.plot_data.current_waypoint = [] ;
    end
    
%% get waypoint
    function waypoint = get_waypoint(HLP,~,~,~)
    % waypoint = get_waypoint(agent_info,obstacles,lookahead_distance)
    %
    % This is the function to be called by the planner replan method. It
    % should return a waypoint, which is usually an (x,y) position or an
    % (x,y,h) pose in SE^2.
        warning('The getWaypoint method is undefined!')
        waypoint = [] ;
    end

%% plotf
    function plot(HLP,plot_format)
        if nargin < 2
            plot_format = '--' ;
        end
        wps = HLP.waypoints ;
        
        % check for plot
        plot_up = check_if_plot_is_available(HLP,'waypoints') ;
        
        if ~isempty(wps)
            if plot_up
                HLP.plot_data.waypoints.XData = wps(1,:) ;
                HLP.plot_data.waypoints.YData = wps(2,:) ;
            else
                HLP.plot_data.waypoints = plot(wps(1,:),wps(2,:),plot_format,'Color',[1 0.5 0]) ;
            end
        end
        
        wp_cur = HLP.current_waypoint ;
        
        if ~isempty(wp_cur)
            if plot_up
                HLP.plot_data.current_waypoint.XData = wp_cur(1) ;
                HLP.plot_data.current_waypoint.YData = wp_cur(2) ;
            else
                HLP.plot_data.current_waypoint = plot(wp_cur(1),wp_cur(2),'^','Color',[1 0.5 0]) ;
            end
        end
    end
    
%% display info
    function vdisp(HLP,s,l)
    % Display a string s if the message's verbose level l is greater
    % than or equal to the waypoint planner's verbose level.
        if nargin < 3
            l = 1 ;
        end
        if HLP.verbose >= l
            if ischar(s)
                disp(['        HLP: ',s])
            else
                disp('        HLP: String not provided!')
            end
        end
    end
end
end