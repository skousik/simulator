classdef straightLineWaypointPlanner < waypointPlanner2D
properties
    goal
    N_waypoints_keep
end
methods
    %% constructor
    function WP = straightLineWaypointPlanner(goal,verbose_level)
        WP@waypointPlanner2D(verbose_level) ;
        WP.goal = goal ;
        WP.waypoints_include_heading = true ;
        WP.N_waypoints = 1 ;
        WP.N_waypoints_keep = 5 ;
        WP.waypoints = nan(3,WP.N_waypoints_keep+1) ;
    end
    
    %% get waypoint
    function waypoint = getWaypoint(WP,agent_pose,~,lookahead_distance)
        % get straight line between agent pose and goal
        p = agent_pose(1:2) ;
        g = WP.goal(1:2) ;
        
        % find length of the straight line and create unit vector
        d = norm(g - p) ;
        u = (g - p)./d ;
        
        % interpolate along line by lookahead distance to create waypoint
        if lookahead_distance > d
            lookahead_distance = d ;
        end
        
        waypoint = p + lookahead_distance.*u ;
        
        % create heading
        h = atan2(g(2) - p(2), g(1) - p(1)) ;
        waypoint = [waypoint ; h] ;
        
        % add new waypoint to list of past waypoints; just keeps the past
        % 5 waypoints around for now
        waypoint_list = [WP.waypoints(:,end-(WP.N_waypoints_keep-1):end), waypoint] ;
        WP.waypoints = waypoint_list ;
    end
    
    %% plotting
    function plotWaypoints(WP,~)
        w = WP.waypoints ;        
        plot(w(1,:),w(2,:),'o','Color',[1 0.5 0]) ;
    end
end
end