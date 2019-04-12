classdef straight_line_HLP < high_level_planner
methods
    function HLP = straight_line_HLP(varargin)
        HLP@high_level_planner(varargin{:}) ;
    end
    
    function waypoint = getWaypoint(HLP,agent_info,~,lookahead_distance)
        if nargin < 4
            lookahead_distance = HLP.default_lookahead_distance ;
        end
        
        g = HLP.goal ;
        z = agent_info(1:2,end) ;
        dir_des = g - z ;
        dir_des = dir_des./norm(dir_des) ;
        waypoint = lookahead_distance.*dir_des +z;
        waypoint(3) = atan2(dir_des(2),dir_des(1));
        
        % update current waypoints
        HLP.current_waypoint = waypoint ;
        HLP.waypoints = [HLP.waypoints, waypoint] ;
        HLP.current_waypoint_index = HLP.current_waypoint_index + 1 ;
    end
end
end