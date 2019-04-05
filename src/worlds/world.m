classdef world < handle
% Class: world
%
% This world superclass contains generic properties and methods to be used
% in the simulator framework. It is 2-D by default.
%
% PROPERTIES
%     dimension             specifies 2-D or 3-D world; default is 2
%
%     start                 (x,y) or (x,y,z) agent start position
%
%     goal                  (x,y) or (x,y,z) desired final location
%
%     goal_radius           radius about the goal; if the agent's center of
%                           mass is within goal_radius of the goal in the
%                           2-norm, then the agent has reached the goal
%
%     bounds                2-D bounds written as [xmin,xmax,ymin,ymax],
%                           or 3-D bounds [xmin,xmax,ymin,ymax,zmin,zmax]
%
%     N_obstacles           number of obstacles (integer)
%
%     obstacles             the obstacles in the world; by default, this is
%                           a 2-by-N array defining polygons by their
%                           vertices, and separated by columns of nans
%
%     current_time          the current time in the world, updated using
%                           agent_info the world does a collision check
%
%     verbose               verbosity level of display; 0 is silent, 1 is
%                           mildly talkative, and 2 or greater is more
%                           talkative
%
%     plot_data             a structure containing the data output of, for
%                           example, the plot function; data can be updated
%                           in the world.plot method instead of re-plotting
%                           entirely, which makes for smooth animations
%
% METHODS
%     world                 constructor method
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
%     get_world_info        returns obstacles within the sensor window of
%                           the agent by default; should be overwritten by
%                           subclasses to be more situation-specific
%
%     goal_check            checks if the agent is within goal_radius of
%                           the goal
%
%     collision_check       checks if the agent has collided with any
%                           obstacles; should be overwritten by subclasses
%                           to be obstacle-specific
%
%     vdisp                 verbose display; prints messages to screen that
%                           if they have a low enough verbosity level

    properties (Access = public)
        dimension = 2 ;
        start = [0;0] ;
        goal = [0;0] ;
        goal_radius = 1 ;
        bounds = Inf*[-1 1 -1 1] ;
        N_obstacles = [] ;
        obstacles = [] ;
        current_time = 0 ;
        verbose = 0 ;
        plot_data
    end
    
    methods
    %% constructor
        function W = world(varargin)
            W = parse_args(W,varargin{:}) ;
        end
        
    %% setup
        function setup(W)
        % Method: setup()
        %
        % Create a random start and goal point, and reset the agent's
        % trajectory contour, which is used for obstacle checking. This
        % is the most generic version of setup, and ignores the
        % N_obstacles input. Subclasses should override this with a
        % situation-specific setup function.

            % get room bounds
            B = W.bounds ;
            
            % generate default start and goal locations
            W.start = zeros(W.dimension,1) ;
            
            if W.dimension == 2
                W.goal = B([2 4]) ;
            elseif W.dimension == 3
                W.goal = B([2 4 6]) ;
            else
                error('World dimension is incorrect! Must be 2 or 3.')
            end
            
            % reset world time index
            W.current_time = 0 ;
        end
        
    %% reset
        function reset(W)
        % Method: reset()
        %
        % Reset the world's current time index to 1. This is generic and
        % can be overwritten in a subclass. This allows a world to reset
        % its timing without creating new start/goal/obstacles, which is
        % the point of the world.setup method.
            W.current_time = 0 ;
        end
        
    %% get world info for planning
        function I = get_world_info(W,~,~)
        % Method: I = get_world_info(agent_info,planner_info)
        %
        % Return all of a world's obstacles, and the obstacle type, in a
        % structure. This is generic and should be overwritten by specific
        % methods that, e.g., check which obstacles are within the robot's
        % sensor radius, or model occlusions.
            I.obstacles = W.obstacles ;
            I.obstacle_type = W.obstacle_type ;
        end
        
    %% goal check
        function out = goal_check(W,agent_info)
        % Method: out = goal_check(agent_info)
        %
        % Checks if the agent's center of mass is within W.goal_radius of
        % W.goal in the 2-norm.
            pos_idx = agent_info.position_indices ;
            z = agent_info.state(pos_idx,:) ;
            out = min(norm(W.goal - z,2)) <= W.goal_radius ;
        end
        
    %% crash check
        function out = collision_check(W,agent_info,check_full_traj_flag)
        % Function: collision_check(agent,check_full_traj_flag)
        %
        % Given an agent in the world, return true if it has crashed into
        % any obstacles.
        
        out = false ;
        
        % TO DO: fix this up for the new obstacle type
%         
%         % by default, don't check the full trajectory
%             if nargin < 3
%                 check_full_traj_flag = false ;
%             end
%         
%         % initialize output (innocent until proven guilty)
%             out = 0 ;
%             
%         % extract agent info
%             xyidx = agent.xy_state_indices ;
%             hidx = agent.heading_state_index ;
%             tidx = W.current_time ;
%             afc = agent.footprint_contour ;
%             
%         % set up obstacles
%             O = [W.obstacles, nan(2,1), boundsToContour(W.bounds)] ;
%             
%         % if the agent has moved, we need to check if it crashed;
%         % alternatively, we could check the entire trajectory for crashes;
%         % finally, if it didn't move, then check the full trajectory
%             if check_full_traj_flag || tidx == length(agent.time)
%                 Z = agent.state ;
%                 T = agent.time ;
%             elseif tidx <= length(agent.time)
%                 try
%                     Z = agent.state(:,tidx-1:end) ; % the '-1' guarantees overlap
%                                                     % with the old trajectory
%                     T = agent.time(:,tidx-1:end) ;
%                 catch
%                     Z = agent.state(:,tidx:end) ; % in case tidx == 1
%                     T = agent.time(:,tidx:end) ;
%                 end
%             else
%                 error(['The world time index is incorrect! It should be ',...
%                        'no greater than the length of time that the ',...
%                        'agent has been active'])
%             end
%             
%             % create the desired time vector; we check for collision every
%             % 10ms by default, in 5s chunks of time
%             dt = T(end) - T(1) ; % total time of traj
%             Nchk = ceil(dt/5) ; % number of checks to make
%             
%             tidx = T(1) ; % start time of each check
%             for chkidx = 1:Nchk
%                 % get the time vector to check
%                 tlog = T >= tidx ;
%                 Tidx = T(tlog) ;
%                 Zidx = Z(:,tlog) ;
%                 
%                 % create the time vector for interpolation
%                 tchk = 0:0.01:5  + tidx ;
%                 tchk = tchk(tchk <= Tidx(end)) ;
%                 
%                 % create the interpolated trajectory
%                 try
%                    Zchk = matchTrajectories(tchk,Tidx,Zidx) ;
%                    
%                    % create a copy of the agent footprint, rotated at each
%                    % point of the trajectory
%                    X = Zchk(xyidx,:) ; % xy positions of trajectory
%                    N = size(X,2) ;
%                    X = repmat(X(:),1,size(afc,2)) ;
%                        % rep'd for each pt of agent
%                        % footprint contour
%                    
%                    if ~isempty(hidx) && (length(agent.footprint) > 1)
%                        % if there is a heading, and the agent is not a circle,
%                        % then rotate each footprint contour
%                        H = Zchk(hidx,:) ;
%                        R = rotmat(H) ;
%                        F = R*repmat(afc,N,1) + X ;
%                    else
%                        % otherwise, just place the footprint contour at each
%                        % point of the trajectory
%                        F = repmat(afc,N,1) + X ;
%                    end
%                    
%                    Fx = [F(1:2:end,:)' ; nan(1,N)] ;
%                    Fy = [F(2:2:end,:)' ; nan(1,N)] ;
%                    F = [Fx(:)' ; Fy(:)'] ;
%                    
%                    % check if the resulting contour intersects the obstacles
%                    [ci,~] = polyxpoly(F(1,:),F(2,:),O(1,:),O(2,:)) ;
%                    
%                    if ~isempty(ci)
%                        out = 1 ;
%                        break
%                    end
%                    
%                    % increment the time index
%                    tidx = tidx + 5 ;
%                 catch
%                    W.vdisp('Check failed, skipping to next portion!',2)
%                    out = -1 ;
%                 end
%             end
% 
%             % update the world time index
%             W.current_time = length(agent.time) ;
        end 
       
    %% plotting
        function plot(W)
            if ~any(isinf(W.bounds))
                axis(W.bounds)
            end
            
            % TO DO: plot start and goal, obstacles, and world bounds
            
%             % plot start and goal
%             xcirc = cos(linspace(0,2*pi)) ;
%             ycirc = sin(linspace(0,2*pi)) ;
%             
%             plot(W.start(1), W.start(2), 'ko')
%             plot(W.goal(1), W.goal(2), 'kx')
%             r = W.goal_radius ;
%             plot(r*xcirc + W.goal(1), r*ycirc + W.goal(2), 'k:')
        end
        
        function plot_at_time(W,t,color)
            if nargin < 3
                color = [1 0 0] ;
            end
            
            W.vdisp('Plotting at a specific time is undefined.',2)
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
