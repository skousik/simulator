classdef RRT_HLP  < high_level_planner
    properties
        timeout = 0.1 ; % seconds
        d_stop = 2 ;
    end
    methods
        %% constructor
        function HLP = RRT_HLP(varargin)
            HLP@high_level_planner(varargin{:}) ;
        end
        
        %% get waypoint
        function wp = get_waypoint(HLP,agent_info,obstacles,lookahead_distance)
            % grow the RRT until the timeout is reached or there are nodes
            % at least HLP.d_stop away from the agent's current position,
            % then return a waypoint that is in the direction of the global
            % goal and the lookahead_distance away from the agent
            
            % get agent's current position
            z = agent_info.position(:,end) ;
            
            % grow tree
            start_tic = tic ;
            t_cur = toc(start_tic) ;
            d_cur = inf ;
            while t_cur < HLP.timeout && d_cur >= HLP.d_stop
                t_cur = toc(start_tic) ;
            end
        end
        
        %% node feasibility check
        function out = node_feasibility_check(HLP,node,obstacles)
            
        end
    end
end