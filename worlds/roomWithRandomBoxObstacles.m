classdef roomWithRandomBoxObstacles < world2D
properties
    obstacles_unseen
    obstacles_seen
    obstacle_size_bounds
    obstacle_rotation_bounds
end

methods
%% constructor
    function W = roomWithRandomBoxObstacles(bounds,goal_radius,...
                                            default_buffer,obs_size,...
                                            obs_rotation_bounds,...
                                            verbose_level)

        if nargin < 6
            verbose_level = 0 ;
        end

        % prep world2D
        W@world2D(bounds,goal_radius,default_buffer,...
                  'CCW_polygons',verbose_level)

        % get bounds
        xlo = bounds(1) ; xhi = bounds(2) ;
        ylo = bounds(3) ; yhi = bounds(4) ;

        if ~exist('obs_size','var')
            olo = max(abs(yhi-ylo),abs(xhi-xlo))/30 ;
            ohi = 2*olo ;
            obs_size = [olo,ohi] ;
        end

        if ~exist('obs_rotation_bounds','var')
            obs_rotation_bounds = [-pi, pi] ;
        end

        W.obstacle_size_bounds = obs_size ;
        W.obstacle_rotation_bounds = obs_rotation_bounds ;
    end
%% setup
    function setup(W,Nobs)
        % call the generic setup function, which resets the time index
        % (this is good practice for any subclass of world2D)
        setup@world2D(W)

        % get room bounds
        B = W.bounds ;
        xlo = B(1) ; xhi = B(2) ; ylo = B(3) ; yhi = B(4) ;

        % get obstacle info
        if nargin < 2
            Nobs = 0 ;
        end
        obs_size = W.obstacle_size_bounds ;
        obs_rotation_bounds = W.obstacle_rotation_bounds ;

        % generate start position on left side of room with initial
        % heading of 0, and make sure it's not too close too the walls
        b = W.default_buffer ;
        xlo = xlo + b ; xhi = xhi - b ;
        ylo = ylo + b ; yhi = yhi - b ;
        s = [0.1*(xhi - xlo).*rand(1) + xlo + 0.1*(xhi - xlo) ;
             (yhi - ylo).*rand(1) + ylo ;
             0 ] ;
        W.start = s ;

        % generate goal position on right side of room
        g = [0.1*(xhi - xlo).*rand(1) + xlo + 0.9*(xhi - xlo) ;
             0.9*(yhi - ylo).*rand(1) + ylo + 0.1*(yhi - ylo) ] ;
        W.goal = g ;

        % generate obstacles around room
        O = nan(2, 6*Nobs) ; % preallocate obstacle matrix;
                                 % note that there will be a final
                                 % column of NaNs, and that the
                                 % room boundary will be included as an
                                 % obstacle
        llo = obs_size(1) ; lhi = obs_size(2) ;
        orlo = obs_rotation_bounds(1) ;
        orhi = obs_rotation_bounds(2) ;
        
        xlo = B(1) ; xhi = B(2) ; ylo = B(3) ; yhi = B(4) ;
        xlo = xlo + b ; xhi = xhi - b ;
        ylo = ylo + b ; yhi = yhi - b ;

        for idx = 1:6:(6*Nobs-1)
            l = (lhi-llo)*rand(1) + llo ; % length
            r = (orhi-orlo)*rand(1) + orlo ; % rotation

            % obstacle rotation
            R = [cos(r) sin(r) ; -sin(r) cos(r)] ;

            % obstacle base
            o = [-l/2  l/2 l/2 -l/2 -l/2 ;
                 -l/2 -l/2 l/2  l/2 -l/2 ] ;

            % obstacle center, which must be at least 2 times the
            % default buffer distance away from the start and goal
            % points
            d_center = 0 ;
            ds_limit = 1 ;
            dg_limit = 3*b ;
            while (d_center < ds_limit || d_center < dg_limit)
                c = [(xhi-xlo)*rand(1) + xlo ;
                     (yhi-ylo)*rand(1) + ylo ] ;
                ds = min(distPointToPoints(s(1:2),c)) ;
                dg = min(distPointToPoints(g,c)) ;
                d_center = min(ds,dg) ;
            end

            O(:,idx:idx+4) = R*o + repmat(c,1,5) ;
        end

        W.obstacles = O ;
        W.obstacles_seen = [] ;
        W.obstacles_unseen = O ;
        W.N_obstacles = Nobs ;
    end
    
    function setupCustomTrial(W,s,g,O,Nobs)
        W.start = s ;
        W.goal = g ;
        W.obstacles = O ;
        W.obstacles_seen = [] ;
        W.obstacles_unseen = O ;
        W.N_obstacles = Nobs ;
    end

%% get nearby obstacles
    function O_out = getNearbyObstacles(W,agent,~,time_indices)
        % Method: getNearbyObstacles
        %
        % This method overrides the default world2D version. This
        % returns all obstacles seen so far by the agent. It also
        % detects obstacles based on closest distance from each
        % obstacle to the trajectory, as opposed to just checking the
        % vertices of the obstacles.

        if nargin < 4
            time_indices = 1:length(agent.time) ;
        end
        
        zcur = agent.state(agent.xy_state_indices,time_indices)' ;
        zcur = round(zcur,6) ; 
        z = unique(zcur,'rows')' ;
        r = agent.sensor_radius ;

        O = W.obstacles_unseen ;

        if ~isempty(O)
            N = size(O,2)/6 ; % number of obstacles still unseen

            indices_seen = [] ;

            O_out = [] ;
            for idx = 1:6:(6*N-1)
                if size(z,2) == 1
                    dToObs = distPointToPolyline(z,O(:,idx:idx+4)) ;
                else
                    dToObs = poly_poly_dist(z(1,:),z(2,:),...
                                O(1,idx:idx+4),O(2,idx:idx+4)) ;
                end
                if any(dToObs <= r)
                   O_out = [O_out, O(:,idx:idx+4),nan(2,1)] ;
                   indices_seen = [indices_seen, idx:idx+5] ;
                end
            end

            % if some obstacles were seen, remove them from O
            O(:,indices_seen) = [] ;
            W.obstacles_unseen = O ;
            W.obstacles_seen = [W.obstacles_seen, O_out] ;
        end

        O_out = W.obstacles_seen ;
    end
end
end