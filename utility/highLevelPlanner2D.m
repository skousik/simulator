classdef highLevelPlanner2D < handle
% Class: highLevelPlanner2D
%
% For use with simulator2D. The "high-level planner" is responsible for
% creating waypoints for an agent to navigate a world. This class is to be
% called within a planner2D object at each planning iteration (i.e., when
% the planner2D.replan method is called); it is not called by the
% simulator2D.run_simulation method. It just turns out that having a
% high-level planner is really useful for most "mid-level" trajectory
% planners, which is what the planner2D class is.

%% properties
properties
    default_lookahead
    start
    goal
    waypoint_reached_radius
    waypoints
    N_waypoints
    current_waypoint
    current_waypoint_index
    waypoints_include_heading
    verbose
end

%% methods
methods
    function WP = highLevelPlanner2D(start,goal,verbose_level)
        WP.start = start ;
        WP.goal = goal ;
        WP.default_lookahead = 1 ;
        WP.waypoints = [start(1:2), goal] ;
        WP.N_waypoints = size(WP.waypoints,2) ;
        WP.waypoint_reached_radius = 1 ;
        WP.waypoints_include_heading = false ;
        WP.current_waypoint = start(1:2) ;
        WP.current_waypoint_index = 1 ;
        WP.verbose = verbose_level ;
    end
    
    function waypoint = getWaypoint(~,~,~,~)
    % waypoint = getWaypoint(agent_pose,obstacles,lookahead_distance)
    %
    % This is the function to be called by the planner2D.replan method. It
    % should return a waypoint, which is usually an (x,y) position or an
    % (x,y,h) pose in SE^2.
        warning('The getWaypoint method is undefined!')
        waypoint = [] ;
    end
    
    function vdisp(WP,s,l)
    % Display a string s if the message's verbose level l is greater
    % than or equal to the planner's verbose level.
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