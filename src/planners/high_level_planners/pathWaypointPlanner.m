classdef pathWaypointPlanner < high_level_planner
properties
    path
    arc_length
    arc_length_avoid
    N_waypoints_keep
end
methods
    %% constructor
    function WP = pathWaypointPlanner(path,avoid_polygon,verbose_level)
        if nargin<3
            verbose_level=0;
            if nargin<2
                avoid_polygon = [];
            end
        end
        
        WP@high_level_planner('verbose',verbose_level) ;
        WP.path = path ;
        WP.arc_length = [0,cumsum(sqrt(diff(WP.path(1,:)).^2+diff(WP.path(2,:)).^2))];
        WP.waypoints_include_heading = true ;
        WP.N_waypoints = 1 ;
        WP.N_waypoints_keep = 5 ;
        WP.waypoints = nan(3,WP.N_waypoints_keep+1) ;
        
        if ~isempty(avoid_polygon)
          [~, xc_is, yc_is] = poly_poly_dist(path(1,:),path(2,:),avoid_polygon(1,:),avoid_polygon(2,:));
          
          path_points_avoid = [xc_is(:,1)';yc_is(:,1)'];
          
          arc_length_avoid = NaN(1,length(path_points_avoid));
          
          for i=1:size(path_points_avoid,2)
              arc_length_avoid(i) = distAlongPolyline(path_points_avoid(:,i),path(1:2,:));
          end
          
          WP.arc_length_avoid = [min(arc_length_avoid),max(arc_length_avoid)];
          
        else
           WP.arc_length_avoid = [Inf,-Inf]; 
        end
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
        
        if sout < WP.arc_length_avoid(2) && sout>WP.arc_length_avoid(1)
            sout = WP.arc_length_avoid(2)+0.01*WP.arc_length(end);
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