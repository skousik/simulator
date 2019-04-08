classdef straight_line_HLP < high_level_planner
methods
    function HLP = straight_line_HLP(varargin)
        HLP@high_level_planner(varargin{:}) ;
    end
    
    function waypoint = get_waypoint(HLP,agent_info,~,lookahead_distance)
        if nargin < 4
            lookahead_distance = HLP.default_lookahead_distance ;
        end
        
        g = HLP.goal ;
        z = agent_info.position(:,end) ;
        dir_des = g - z ;
        dir_des = dir_des./norm(dir_des) ;
        waypoint = lookahead_distance.*dir_des ;
        
        % update current waypoints
        HLP.current_waypoint = waypoint ;
        HLP.waypoints = [HLP.waypoints, waypoint] ;
        HLP.current_waypoint_index = HLP.current_waypoint_index + 1 ;
    end
end
end