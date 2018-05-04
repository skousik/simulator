classdef segwayRRTplanner < planner2D
    properties        
	% implementation-specific properties
        dt_edge % RRT time discretization of each edge (trajectory between nodes)
        T_edge % RRT duration of each edge
        iter_max % RRT iteration max
        wpf % A* planner
        lookahead_distance % used by wpf to make waypoints; can be a scalar
                           % or can be set to the string 'dynamic' in which
                           % the lookahead is based on current + max speed
        goal_radius
        spinflag % spin to face the first waypoint in the first iteration
        t_min % min length of plan
        t_max % max length of plan
        T_old
        U_old
        Z_old
    end
    
    methods
    %% constructor
    function P = segwayRRTplanner(t_move, timeout, t_min, t_max, dt_edge, T_edge, iter_max, ...
                                  buffer_size, lookahead_distance, verbose_level,...
                                  name)
        
        if nargin < 8
            buffer_size = 0.5 ;
        end
        
        if nargin < 9
            lookahead_distance = 'dynamic' ;
        end                              
                            
        if nargin < 10
            verbose_level = 0 ;
        end
        
        if nargin < 11
            name = 'RRT' ;
        end
        
        P@planner2D(timeout,verbose_level) ;
        P.t_move = t_move ;
        P.t_min = t_min ; % typically this is t_plan + t_stop
        P.t_max = t_max ;
        P.name = name ;
        P.dt_edge = dt_edge ;
        P.T_edge = T_edge ;
        P.iter_max = iter_max ;
        P.default_buffer = buffer_size ;
        P.lookahead_distance = lookahead_distance ;
    end
    
    %% setup
    function setup(P,~,W)
        % get world bounds
%         P.default_buffer = A.footprint + 0.02 ;
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
                            
        P.T_old = 0 ;
        P.U_old = zeros(2,1) ;
        P.Z_old = [W.start ; 0 ; 0] ;
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
        
        
        if ischar(P.lookahead_distance) && strcmp(P.lookahead_distance,'dynamic')
            lkhd = z0(end) + A.vmax ;
            wp = P.wpf.makeWaypoint(lkhd) ;
        elseif isnumeric(P.lookahead_distance)
            wp = P.wpf.makeWaypoint(P.lookahead_distance) ;
        else
            error('Lookahead distance setting is invalid!')
        end
        
        P.current_waypoint = wp ;
        
        % run planner
        if P.spinflag
            % spin in place to face the first waypoint on the first
            % iteration
            P.vdisp('Spinning in place!', 4)
            P.spinflag = false ;
            
            % this is the heading from the current position to the
            % waypoint, and no longer rotated by the segway's current
            % heading - this will cause the robot to spin a lot more, which
            % makes it less conservative
            h2wp = atan2(wp(2)-xy(2),wp(1)-xy(1)) ; % - z0(3)
            
            wdes = max(min(sign(h2wp),A.wmax),-A.wmax) ;
            Tout = [0 1] ;
            Uout = [wdes wdes ; 0 0] ;
            Zout = [] ;
        else
            % initialize vertices
            V = [[z0;0] , nan(length(z0(:))+1,P.iter_max - 1)] ;
            NV = 1 ; % vertex count

            U = zeros(2,P.iter_max) ; % keep track of control inputs

            NE = 0 ; % edge count

            Adj = sparse(P.iter_max,P.iter_max) ; % graph adjacency matrix

            % get timing
            timeout = P.timeout ;

            % run RRT
            P.vdisp('Running RRT.', 4)
            d = Inf ;
            t_start = tic ;
            t_timeout = toc(t_start) ;
            icur = 1 ;
            
            while icur < P.iter_max && t_timeout <= timeout && d > A.footprint
                % choose random vertex
    %             v_near_idx = randi(NV) ;
                v_near_idx = round(randRange(1,NV,3*NV/4,1*NV/4)) ;
                z_near = V(1:5,v_near_idx) ;
                x_near = V(1,v_near_idx) ;
                y_near = V(2,v_near_idx) ;
                h_near = V(3,v_near_idx) ;
                t_near = V(end,v_near_idx) ;
                
                % if the current node is within t_max, use it to spawn a
                % new node
                if t_near < P.t_max
                    % generate random desired control input that prefers to turn
                    % towards the waypoint
                    bias = atan2(wp(2)-y_near, wp(1)-x_near) ;
                    wdes = max(min((A.wmax/2)*randn + sin(bias-h_near),A.wmax),-A.wmax) ;
                    vdes = max(min(((A.vmax/4)*randn + A.vmax/2), A.vmax), 0) ;

                    % create new vertex
                    Tin = 0:P.dt_edge:P.T_edge ;
                    Uin = repmat([wdes;vdes],1,size(Tin,2)) ;
                    [T_new,Z_new] = A.odesolver(@(t,z) A.dynamics(t,z,Tin,Uin,[]),...
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
                            % check if new edge intersects obstacle, unless
                            % the robot is /in/ an obstacle's (buffered)
                            % footprint (in which case, we allow it to
                            % create nodes to get away from the obstacle
                            % up to time t_move)
                            xcheck = Z_new(1,:) ;
                            ycheck = Z_new(2,:) ;

%                             [in1,~] = polyxpoly(xcheck',ycheck',O(1,:)',O(2,:)') ;
                            [in2,~] = inpolygon(xcheck',ycheck',O(1,:)',O(2,:)') ;

%                             if (~isempty(in1) || any(in2))
                            if any(in2)
                                    traj_feasible = false ;
                            end
                        end

                        if traj_feasible
                            % if not in an obstacle, add to list
                            NV = NV + 1 ;
                            NE = NE + 1 ;

                            V(:,NV) = [z_new ; t_near + T_new(end)] ;
                            U(:,NE) = [wdes ; vdes] ;                    

                            % update adjacency matrix
                            Adj(v_near_idx,NV) = distPointToPoints(z_new(1:2),[x_near;y_near]) ;
                        end

                        % check if reached waypoint
                        d = min(distPointToPoints(wp,V(1:2,:))) ;

%                         % if trying to reach the global goal, use the world's goal
%                         % radius instead of the robot's footprint
%                         if norm(wp - P.wpf.g) < A.footprint
%                             d = d - P.goal_radius ;
%                         end
                    end % from checking if vertex is in world bounds

                    icur = icur + 1 ;
                end % from: if t_near < P.t_max
                
                t_timeout = toc(t_start) ;
            end

            % extract shortest path
            [~,v_goal_idx] = min(distPointToPoints(wp,V(1:2,:))) ;
            [~,path,~] = graphshortestpath(Adj,1,v_goal_idx) ;

            % create output time
%             Tout = 0:P.T_edge:P.T_edge*(length(path)-1) ;
            Tout = V(end,path) ;
            
            % if time is not long enough, brake
            if Tout(end) >= P.t_min || norm(wp - P.wpf.g) < A.footprint
                P.vdisp('RRT found successful path!', 4) ;
                % create input
                Uout = U(:,path) ;  % the last input is zeros, which
                                     % corresponds to the last time
                                     % step, i.e. the final input that
                                     % doesn't get used

                % create anticipated path
                Zout = V(1:5,path) ;
            else
%                 P.vdisp('Braking along previous trajectory!', 4)
%                 
%                 T = P.T_old ;
%                 U = P.U_old ;
%                 Z = P.Z_old ;
%                 
%                 % find closeset point in Zold to current state
%                 xy_old = Z(1:2,:) ;
%                 [~,idx] = min(distPointToPoints(xy,xy_old)) ;
%                 
%                 % get the xy difference and shift Z by that much
%                 dxy = xy - xy_old(:,1) ;
%                 Z(1:2,:) = Z(1:2,:) + repmat(dxy,1,size(Z,2)) ;
% 
%                 T = T(idx:end) - T(idx) ;
%                 U = U(:,idx:end) ;
%                 Z = Z(:,idx:end) ;
%                 
%                 % make sure that T is long enough
%                 if T(end) < P.t_max
%                     T = [T(1:end-1), linspace(T(end),P.t_max,10)] ;
%                     U = [U, zeros(2,9)] ;
%                     Z = [Z, repmat(Z(:,end),1,9)] ;
%                 end
%                 
%                 % create braking trajectory
%                 [Tout,Uout,Zout] = A.makeBrakingTrajectory(T,U,Z) ;
                
                % set spinflag for next iteration
                P.spinflag = true ;
                Tout = [] ;
                Uout = [] ;
                Zout = [] ;
            end
            
            P.T_old = Tout ;
            P.U_old = Uout ;
            P.Z_old = Zout ;
            
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
        if ~isempty(w)
            plot(w.w(1,:),w.w(2,:),'--')
            plot(P.current_waypoint(1),P.current_waypoint(2),'ro')
        end
        
        % obstacles
        O = P.current_obstacles ;
        if ~isempty(O)
            plot(O(1,:),O(2,:),'-','Color',[1 0.5 0.5]) ;
        end
    end
    end
end
