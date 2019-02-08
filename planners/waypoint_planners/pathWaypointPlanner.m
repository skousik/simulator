classdef pathWaypointPlanner < waypointPlanner2D
properties
    path
    arc_length
    N_waypoints_keep
end
methods
    %% constructor
    function WP = pathWaypointPlanner(path,verbose_level)
        WP@waypointPlanner2D(verbose_level) ;
        WP.path = path ;
        WP.arc_length = [0,cumsum(sqrt(diff(WP.path(1,:)).^2+diff(WP.path(2,:)).^2))];
        WP.waypoints_include_heading = true ;
        WP.N_waypoints = 1 ;
        WP.N_waypoints_keep = 5 ;
        WP.waypoints = nan(3,WP.N_waypoints_keep+1) ;
    end
    
    %% get waypoint
    function waypoint = getWaypoint(WP,agent_pose,~,lookahead_distance)
        % get straight line between agent pose and goal
        p = agent_pose(1:2) ;
        scur = distAlongPolyline(p,WP.path(1:2,:));
        sout = scur + lookahead_distance;
        
        if sout>WP.arc_length(end)
            sout=WP.arc_length(end);
        end
        
        
        waypoint = interp_with_angles(WP.arc_length',WP.path',sout,3,'pchip')';
        
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