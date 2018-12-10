classdef waypointPlanner2D < handle
% Class: waypointPlanner2D
%
% For use with simulator2D. This "high-level planner" is responsible for
% creating waypoints for an agent to navigate a world. This class is to be
% called within a planner2D object at each planning iteration (i.e., when
% the planner2D.replan method is called); it is not called by the
% simulator2D.run_simulation method. It just turns out that having a
% high-level planner is really useful for most "mid-level" trajectory
% planners, which is what the planner2D class is.

%% properties
properties
    default_lookahead
    waypoint_reached_radius
    waypoints
    N_waypoints
    current_waypoint
    current_waypoint_index
    waypoints_include_heading
    verbose
end

methods
%% constructor
    function WP = waypointPlanner2D(verbose_level)
        if nargin < 1
            verbose_level = 0 ;
        end
        
        WP.default_lookahead = 1 ;
        WP.waypoints = [] ;
        WP.N_waypoints = 0 ;
        WP.waypoint_reached_radius = 1 ;
        WP.waypoints_include_heading = false ;
        WP.current_waypoint = [] ;
        WP.current_waypoint_index = [] ;
        WP.verbose = verbose_level ;
    end
    
%% get waypoint
    function waypoint = getWaypoint(~,~,~,~)
    % waypoint = getWaypoint(agent_pose,obstacles,lookahead_distance)
    %
    % This is the function to be called by the planner2D.replan method. It
    % should return a waypoint, which is usually an (x,y) position or an
    % (x,y,h) pose in SE^2.
        warning('The getWaypoint method is undefined!')
        waypoint = [] ;
    end

%% plot waypoints
    function plotWaypoints(WP,plotformat)
        if nargin < 2
            plotformat = '--' ;
        end        
        w = WP.waypoints ;        
        plot(w(1,:),w(2,:),plotformat,'Color',[1 0.5 0]) ;
    end
    
%% display info
    function vdisp(WP,s,l)
    % Display a string s if the message's verbose level l is greater
    % than or equal to the waypoint planner's verbose level.
        if nargin < 3
            l = 1 ;
        end
        if WP.verbose >= l
            if ischar(s)
                disp(['        WP: ',s])
            else
                disp('        WP: String not provided!')
            end
        end
    end
end
end