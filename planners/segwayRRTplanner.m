classdef segwayRRTplanner < planner2D
    properties        
	% implementation-specific properties
        dt % RRT edge time discretization
        T % RRT edge time horizon
        iter_max % RRT iteration max
        wpf % A* planner
        lookahead_distance % used by wpf to make waypoints
        goal_radius
        spinflag % spin to face the first waypoint in the first iteration
    end
    
    methods
    %% constructor
    function P = segwayRRTplanner(timeout, t_move, dt, T, iter_max, ...
                                  lookahead_distance, verbose_level,...
                                  name)
        if nargin < 7
            verbose_level = 0 ;
        end
        
        if nargin < 8
            name = 'RRT' ;
        end
        
        P@planner2D(timeout,verbose_level) ;
        P.t_move = t_move ;
        P.name = name ;
        P.timeout = timeout ;
        P.dt = dt ;
        P.T = T ;
        P.iter_max = iter_max ;
        P.lookahead_distance = lookahead_distance ;
    end
    
    %% setup
    function setup(P,A,W)
        % get world bounds
        P.default_buffer = A.footprint + 0.01 ;
        b = P.default_buffer ;
        P.bounds = W.bounds + [b -b b -b] ;
        P.goal_radius = W.goal_radius ;
        
        P.spinflag = true ;

        % set up waypoint finder
        s = W.start(1:2) ;
        g = W.goal ;
        P.wpf = waypointFinder2D(s, g, 'gridSpacing', 0.25,...
                                'gridtype','rect','costMax',1000,...
                                'costIncrease',100,'costDecrease',0,...
                                'pathDiscount',0,...
                                'gridConnectivity',9,...
                                'gridBounds',P.bounds) ;
    end
    
    function update(P,A,W)
        % Function: update (for RRTdubins planner)
        %
        % This needs to be run when the world's setup function is run, to
        % update the planner with any changes made to the world
        P.setup(A,W) ;
    end
    
    %% replan
    function [Tout,Uout,Zout] = replan(P,A,O)
        % get initial condition from agent
        z0 = A.state(:,end) ;
        xy = z0(A.xy_state_indices) ;
        
        % buffer obstacles (Note: this assumes the obstacles are boxes!)
        if ~isempty(O)
            O = bufferBoxObstacles(O,P.default_buffer) ;
        end
        P.current_obstacles = O ;

        % get bounds, which are already buffered
        B = P.bounds ;

        % create waypoints based on current obstacles and agent position
        P.wpf.z = z0(A.xy_state_indices) ;
        P.wpf.updateGraphWeights(O) ;
        P.wpf.getWaypoints(true) ;
        wps = P.wpf.w ;
        wp = P.wpf.makeWaypoint(P.lookahead_distance) ;
        P.current_waypoint = wp ;
        
        % run planner
        if P.spinflag
            % spin in place to face the first waypoint on the first
            % iteration
            P.spinflag = false ;
            h2wp = atan2(wp(2)-xy(2),wp(1)-xy(1)) ;
            wdes = max(min(h2wp,A.wmax),-A.wmax) ;
            Tout = [0 1] ;
            Uout = [wdes wdes ; 0 0] ;
            Zout = [] ;
        else
            % initialize vertices
            V = [z0 , nan(length(z0(:)),P.iter_max - 1)] ;
            NV = 1 ; % vertex count

            U = zeros(2,P.iter_max) ; % keep track of control inputs

            NE = 0 ; % edge count

            Adj = sparse(P.iter_max,P.iter_max) ; % graph adjacency matrix

            % get timing
            t_max = P.timeout ;

            % run RRT
            d = Inf ;
            t_start = tic ;
            t_timeout = toc(t_start) ;
            icur = 1 ;
            while icur < P.iter_max && t_timeout <= t_max && d > A.footprint
                % choose random vertex
    %             v_near_idx = randi(NV) ;
                v_near_idx = round(randRange(1,NV,3*NV/4,1*NV/4)) ;
                z_near = V(:,v_near_idx) ;
                x_near = V(1,v_near_idx) ;
                y_near = V(2,v_near_idx) ;
                h_near = V(3,v_near_idx) ;

                % generate random desired control input that prefers to turn
                % towards the goal
    %             if all(wp == P.wpf.g)
    %                 % if trying to get to the global goal, use larger variety
    %                 % of steering inputs without a bias
    %                 wdes = 2*A.wmax*rand(1) - A.wmax ;
    %             else
                bias = atan2(wp(2)-y_near, wp(1)-x_near) ;
                wdes = max(min((A.wmax/2)*randn + sin(bias-h_near),A.wmax),-A.wmax) ;
    %             end
                vdes = max(min(((A.vmax/4)*randn + A.vmax/2), A.vmax), 0) ;

                % create new vertex
                Tin = 0:P.dt:P.T ;
                Uin = repmat([wdes;vdes],1,size(Tin,2)) ;
                [~,Z_new] = A.odesolver(@(t,z) A.dynamics(t,z,Tin,Uin,[]),...
                                  Tin,z_near) ;
                Z_new = Z_new' ;
                z_new = Z_new(:,end) ;

                x_new = z_new(1) ; y_new = z_new(2) ;

                % check if new vertex is inside world bounds
                if (x_new >= B(1) && x_new <= B(2)) && (y_new >= B(3) && y_new <= B(4))
                    % check if vertex is close enough to waypoints
                    d_near = min(distPointToPoints([x_new;y_new],wps)) ;
                    traj_feasible = true ;

                    if d_near > 2*A.footprint
                        traj_feasible = false ;
                    elseif ~isempty(O)
                        % check if new edge intersects obstacle
                        xcheck = Z_new(1,:) ;
                        ycheck = Z_new(2,:) ;

                        [in1,~] = polyxpoly(xcheck',ycheck',O(1,:)',O(2,:)') ;
                        [in2,~] = inpolygon(xcheck',ycheck',O(1,:)',O(2,:)') ;

                        if ~isempty(in1) || any(in2)
                            traj_feasible = false ;
                        end
                    end

                    if traj_feasible
                        % if not in an obstacle, add to list
                        NV = NV + 1 ;
                        NE = NE + 1 ;

                        V(:,NV) = z_new ;
                        U(:,NE) = [wdes ; vdes] ;                    

                        % update adjacency matrix
                        Adj(v_near_idx,NV) = distPointToPoints(z_new(1:2),[x_near;y_near]) ;
                    end

                    % check if reached waypoint
                    d = min(distPointToPoints(wp,V(1:2,:))) ;

                    % if trying to reach the global goal, use the world's goal
                    % radius instead of the robot's footprint
                    if norm(wp - P.wpf.g) < A.footprint
                        d = d - P.goal_radius ;
                    end
                end

                icur = icur + 1 ;
                t_timeout = toc(t_start) ;
            end

            % extract shortest path
            [~,v_goal_idx] = min(distPointToPoints(wp,V(1:2,:))) ;
            [~,path,~] = graphshortestpath(Adj,1,v_goal_idx) ;

            % create input
            Uout = U(:,path) ;  % the last input is zeros, which
                                 % corresponds to the last time
                                 % step, i.e. the final input that
                                 % doesn't get used

            % create output time
            Tout = 0:P.T:P.T*(length(path)-1) ;

            % create anticipated path
            Zout = V(:,path) ;
            
            if ~isempty(Zout)
                P.xy_plan = Zout(A.xy_state_indices,:) ;
            else
                P.xy_plan = [] ;
            end
        end
    end
    
    %% plotting
    function plotInLoop(P,n,c)
        if nargin < 3
            c = [0 0 1] ;
        end
        
        plotInLoop@planner2D(P,n,c)
        
        figure(n)
        
        % waypoint finder
        w = P.wpf ;
        plot(w.w(1,:),w.w(2,:),'--')
        plot(P.current_waypoint(1),P.current_waypoint(2),'ro')
    end
    end
end
