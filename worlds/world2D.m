classdef world2D < handle
% Class: world2D
%
% This superclass contains the generic functions for getting obstacles and
% checking the path of an agent in the simulator2D framework. Obstacles can
% be of various types, to be created by subclasses of world2D.
%
% Currently, the only type supported is obstacles as 2-by-N points
% representing counter-clockwise polygonal contours, with distinct
% obstacles separated by columns of NaNs. These obstacles can be time
% varying or static, depending on the subclass implementation.
%
% PROPERTIES
%     start                 (x,y,h) coordinates in SE2 of the robot's start
%                           location and heading
%
%     goal                  (x,y) desired final/goal location of the robot
%
%     goal_radius           radius about the goal; if the robot arrives
%                           anywhere in this area, it is said to have
%                           reached the goal
%
%     bounds                [xmin,xmax,ymin,ymax] 2-D bounds of the world
%
%     bounds_as_contour     the world bounds expressed as a CW polygon
%                           inside a CCW polygon, to allow the world bounds
%                           to be checked by functions like polyxpoly and
%                           inpolygon
%
%     N_obstacles           number of obstacles (scalar)
%
%     obstacles             the obstacles in the world; for static
%                           obstacles, this is a 2-by-N list of CCW
%                           polygons separated by NaNs; for dynamic
%                           obstacles, this is a N_obstacles-by-1 structure
%                           where each entry is an obstacle object
%
%     obstacle_type         either 'static' or 'dynamic'
%
%     default_buffer        a scalar amount that the world expects to
%                           buffer obstacles so that a robot can avoid
%                           collisions while navigating the world;
%                           typically the radius of the robot
%
%     current_time_index    an integer representing the current simulation
%                           time of the agent; this is used by world2D as
%                           current_time = agent.time(current_time_index)
%
%     verbose               verbosity level of display; 0 is silent, 1 is
%                           mildly talkative, and 2 or greater is very
%                           talkative
%
% METHODS
%     world2D               constructor method
%
%     setup                 creates start, goal, and obstacles; since
%                           obstacle configurations are typically
%                           randomized, this is usually called each time
%                           before running a simulation to present all of
%                           the simulator2D planners with a new world
%
%     reset                 resets the current time index to 1 (used when
%                           the agent is reset to the start location,
%                           usually); is usually overwritten by subclasses
%
%     getNearbyObstacles    returns obstacles within the sensor window of
%                           the agent by default; should be overwritten by
%                           subclasses to be more situation-specific
%
%     goalCheck             checks if the agent is within goal_radius of
%                           the goal
%
%     crashCheck            checks if the agent has collided with any
%                           obstacles; should be overwritten by subclasses
%                           to be obstacle-specific
%
%     vdisp                 verbose display; prints messages to screen that
%                           if they have a low enough verbosity level

    properties
        start
        goal
        goal_radius
        bounds
        bounds_as_contour
        N_obstacles
        obstacles
        obstacle_type
        default_buffer
        current_time_index
        verbose
    end
    
    methods
    %% constructor
        function W = world2D(bounds,goal_radius,default_buffer,...
                             obstacle_type, verbose_level)
            if nargin < 5
                verbose_level = 0 ;
            end
            
            if nargin < 4
                obstacle_type = 'static' ;
            end
            
            if nargin < 3
                default_buffer = 1 ;
            end
            
            if nargin < 2
                goal_radius = 1 ;
            end
            
            if nargin < 1
                bounds = Inf*[-1 1 -1 1] ;
            end
            
            W.obstacle_type = obstacle_type ;
            W.goal_radius = goal_radius ;
            W.default_buffer = default_buffer ;            
            
            % generate room bounds as an obstacle
            xlo = bounds(1) ; xhi = bounds(2) ;
            ylo = bounds(3) ; yhi = bounds(4) ;
            
            Oin = [xlo, xhi, xhi, xlo, xlo ;
                   ylo, ylo, yhi, yhi, ylo] ;
            
            xlo = xlo - .1 ; xhi = xhi + .1 ;
            ylo = ylo - .1 ; yhi = yhi + .1 ;
            
            Oout = [xlo, xhi, xhi, xlo, xlo ;
                    ylo, ylo, yhi, yhi, ylo] ;
            
            Obounds = [Oin(:,end:-1:1), nan(2,1), Oout] ;
            
            % default settings
            W.verbose = verbose_level ;
            W.start = [0;0] ;
            W.goal = [0;0] ;
            W.bounds = bounds ;
            W.bounds_as_contour = Obounds ;
            W.obstacles = [] ;
            W.N_obstacles = 0 ;
            W.current_time_index = 1 ; % initializes at t = 0
        end
        
        %% setup
        function setup(W)
        % Method: setup()
        %
        % Create a random start and goal point, and reset the robot's
        % trajectory contour, which is used for obstacle checking. This
        % is the most generic version of setup, and ignores the
        % N_obstacles input. Subclasses should override this with a
        % situation-specific setup function.

            % get room bounds
            B = W.bounds ;
            xlo = B(1) ; xhi = B(2) ; ylo = B(3) ; yhi = B(4) ;
            
            % generate random start and goal locations
            W.start = [randRange(xlo,xhi) ; randRange(ylo,yhi) ; 0] ;
            W.goal = [randRange(xlo,xhi) ; randRange(ylo,yhi)] ;
            
            % reset world time index
            W.current_time_index = 1 ;
        end
        
        %% reset
        function reset(W,~)
        % Method: reset(planner)
        %
        % Resets the world's current time index to 1. Planners are also
        % passed to this method, so that world can extract necessary info
        % from each planner.
            W.current_time_index = 1 ;
        end
        
        %% simple obstacle scanning
        function O = getNearbyObstacles(W,agent,~,time_indices)
        % Method: O = getNearbyObstacles(agent,planner,time_indices)
        %
        % Given an agent object, use its location and sensor radius to
        % find all obstacles in its vicinity. This is the most generic
        % version of this method, and should usually be overridden by a
        % more application-specific version in a specific subclass of
        % world2D.
        
            if strcmp(W.obstacle_type,'static')
                if nargin < 4
                    time_indices = 1:size(agent.state,2) ;
                end

                xyidx = agent.xy_state_indices ;
                z = agent.state(xyidx,time_indices) ;
                r = agent.sensor_radius ;

                O = W.obstacles ;

                if ~isempty(O)
                    dlog = (distPointToPoints(z,O) <= r) | isnan(O(1,:)) ;
                    O = O(:,dlog) ;
                end
            else
                error(['Please write a custom function to get ',...
                       'nearby obstacles.'])
            end
        end
        
        %% goal check
        function out = goalCheck(W,agent)
            xyidx = agent.xy_state_indices ;
            Z = agent.state(xyidx,:) ;
            out = min(distPointToPoints(W.goal,Z)) <= W.goal_radius ;
        end
        
        %% crash check
        function out = crashCheck(W,agent, check_full_traj_flag)
        % Function: crashCheck(agent)
        %
        % Given an agent in the world, return true if it has crashed into
        % any obstacles
        
        % by default, don't check the full trajectory
            if nargin < 3
                check_full_traj_flag = false ;
            end
        
        % initialize output (innocent until proven guilty)
            out = 0 ;
            
        % extract agent info
            xyidx = agent.xy_state_indices ;
            hidx = agent.heading_state_index ;
            tidx = W.current_time_index ;
            afc = agent.footprint_contour ;
            
        % set up obstacles
            O = [W.obstacles, nan(2,1), boundsToContour(W.bounds)] ;
            
        % if the agent has moved, we need to check if it crashed;
        % alternatively, we could check the entire trajectory for crashes;
        % finally, if it didn't move, then check the full trajectory
            if check_full_traj_flag || tidx == length(agent.time)
                Z = agent.state ;
                T = agent.time ;
            elseif tidx <= length(agent.time)
                try
                    Z = agent.state(:,tidx-1:end) ; % the '-1' guarantees overlap
                                                    % with the old trajectory
                    T = agent.time(:,tidx-1:end) ;
                catch
                    Z = agent.state(:,tidx:end) ; % in case tidx == 1
                    T = agent.time(:,tidx:end) ;
                end
            else
                error(['The world time index is incorrect! It should be ',...
                       'no greater than the length of time that the ',...
                       'agent has been active'])
            end
            
            % create the desired time vector; we check for collision every
            % 10ms by default, in 5s chunks of time
            dt = T(end) - T(1) ; % total time of traj
            Nchk = ceil(dt/5) ; % number of checks to make
            
            tidx = T(1) ; % start time of each check
            for chkidx = 1:Nchk
                % get the time vector to check
                tlog = T >= tidx ;
                Tidx = T(tlog) ;
                Zidx = Z(:,tlog) ;
                
                % create the time vector for interpolation
                tchk = 0:0.01:5  + tidx ;
                tchk = tchk(tchk <= Tidx(end)) ;
                
                % create the interpolated trajectory
                try
                   Zchk = matchTrajectories(tchk,Tidx,Zidx) ;
                   
                   % create a copy of the agent footprint, rotated at each
                   % point of the trajectory
                   X = Zchk(xyidx,:) ; % xy positions of trajectory
                   N = size(X,2) ;
                   X = repmat(X(:),1,size(afc,2)) ;
                       % rep'd for each pt of agent
                       % footprint contour
                   
                   if ~isempty(hidx) && (length(agent.footprint) > 1)
                       % if there is a heading, and the agent is not a circle,
                       % then rotate each footprint contour
                       H = Zchk(hidx,:) ;
                       R = rotmat(H) ;
                       F = R*repmat(afc,N,1) + X ;
                   else
                       % otherwise, just place the footprint contour at each
                       % point of the trajectory
                       F = repmat(afc,N,1) + X ;
                   end
                   
                   Fx = [F(1:2:end,:)' ; nan(1,N)] ;
                   Fy = [F(2:2:end,:)' ; nan(1,N)] ;
                   F = [Fx(:)' ; Fy(:)'] ;
                   
                   % check if the resulting contour intersects the obstacles
                   [ci,~] = polyxpoly(F(1,:),F(2,:),O(1,:),O(2,:)) ;
                   
                   if ~isempty(ci)
                       out = 1 ;
                       break
                   end
                   
                   % increment the time index
                   tidx = tidx + 5 ;
                catch
                   W.vdisp('Check failed, skipping to next portion!',2)
                   out = -1 ;
                end
            end

            % update the world time index
            W.current_time_index = length(agent.time) ;
        end 
       
%% plotting
        function plot(W)
            W.plotInLoop(1)
            axis equal
        end
        
        function plotInLoop(W,n)
            figure(n) ; hold on ;
            if ~any(isinf(W.bounds))
                axis(W.bounds)
            end
            
            % plot start and goal
            xcirc = cos(linspace(0,2*pi)) ;
            ycirc = sin(linspace(0,2*pi)) ;
            
            plot(W.start(1), W.start(2), 'ko')
            plot(W.goal(1), W.goal(2), 'kx')
            r = W.goal_radius ;
            plot(r*xcirc + W.goal(1), r*ycirc + W.goal(2), 'k:')
            
            % plot obstacles
            O = W.obstacles ;
            if ~isempty(O) && strcmp(W.obstacle_type,'static')
                plot(O(1,:),O(2,:),'r-')
            end
            
            % plot world bounds
            if ~any(isinf(W.bounds))
                B = W.bounds_as_contour ; % [xmin xmax ymin ymax]
                plot(B(1,:),B(2,:),'r')
            end
        end
        
        function plotResults(W,n)
            W.plotInLoop(n) ;
        end
        
%% utility
        function vdisp(W,s,l)
        % Display a string s if the message's verbose level l is greater
        % than or equal to the planner's verbose level.
            if nargin < 3
                l = 1 ;
            end
            if W.verbose >= l
                if ischar(s)
                    disp(['    W: ',s])
                else
                    disp('    W: String not provided!')
                end
            end
        end
    end
end
