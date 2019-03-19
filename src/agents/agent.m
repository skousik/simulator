classdef agent < handle
% Class: agent
%
% Generic point-mass agent with zero dynamics as default. Other agents
% should inherit this as a superclass.
%
% PROPERTIES:
%   verbose              the verbosity level of the agent; 0 is silent,
%                        1 is mildly talkative, 2 is very talkative, etc.
%
%   state                n_states-by-N vector of the states at each time
%                        point indicated by the "time" property
%
%   time                 1-by-N vector indicating the time at which the
%                        agent is at each state
%
%   input                n_inputs-by-M vector indicating the input applied
%                        by the agent, assumed to be zero-order hold, with
%                        input timing given by the "input_time" property
%
%   input_time           1-by-M vector indicating the timing of the input
%
%   n_states             the number of states the agent has
%
%   n_inputs             the number of inputs the agent has
%
%   position_indices     indices of the position coordinates as a 1-by-2
%                        vector for 2-D or 1-by-3 for 3-D; by default, this
%                        is [1,2] or [1,2,3]
%
%   orientation_indices  indices of yaw (for 2-D) and pitch and roll (3-D);
%                        by default [3], or [3,4,5]]                        
%
%   sensor_radius        a scalar indicating the sensor radius of the
%                        agent, which can be used by the world to determine
%                        which obstacles and portions of the world to
%                        reveal to the agent
%
%   plot_sensor_radius   a boolean that determines whether or not to plot
%                        the sensor radius when plot_in_loop is called
%
% METHODS:
%   agent                the constructor method, which establishes the
%                        default properties
%
%   reset                by default, sets the agent's state to the provided
%                        state and its inputs and time to zero
%
%   move                 takes in a control input and associated time, and
%                        potentially a reference trajectory, and moves the
%                        agent based on this information; updates the
%                        agent's state, time, input, and input_time
%
%   predict              same as move, but doesn't update the state, time,
%                        input, or input time; instead, returns a structure
%                        containing the prediction with appropriate field
%                        names
%
%   stop                 by default, calls "move" with zero control inputs
%                        for 1 second; should be replaced in a subclass
%                        with a "braking" functionality, typically
%                        implemented by calling "move"
%
%   dynamics             the differential equation describing the agent's
%                        motion given an input, input timing, and
%                        optionally a reference trajectory; by default
%                        zero, and should be overwritten in a subclass
%
%   ode_solver            a solver used by the agent; by default ode45 is
%                        used, and should be sufficient for most agents;
%                        overwriting this in a subclass is useful for
%                        implementing RK4 or some other fixed-step
%                        integration method
%
%   plot_in_loop           plots the agent's trajectory and footprint; this
%                        method is called once per iteration of the
%                        simulator

    properties
        state
        time
        input
        input_time
        n_states
        n_inputs
        position_indices
        orientation_indices
        sensor_radius
        verbose
        plot_sensor_radius
        plot_time_step
    end
    
    methods
    %% constructor
        function A = agent(varargin)
        % method: A = agent(verbose_level,varargin)
        %
        % This method creates an agent object. It takes in string/argument
        % pairs, where the strings are the names of the agent default
        % properties.
        
            if mod(length(varargin),2) == 1
                verbose_level = varargin{1} ;
                varargin = varargin(2:end) ;
            else
                verbose_level = 0 ;
            end
            A.verbose = verbose_level ;
       
            for idx = 1:2:length(varargin)
                switch varargin{idx}
                    case 'sensor_radius'
                        sensor_radius = varargin{idx+1} ;
                    case 'position_indices'
                        position_indices = varargin{idx+1} ;
                    case 'n_states'
                        n_states = varargin{idx+1} ;
                    case 'n_inputs'
                        n_inputs = varargin{idx+1} ;   
                    case 'orientation_indices'
                        orientation_indices = varargin{idx+1} ;
                    case 'plot_sensor_radius'
                        plot_sensor_radius = varargin{idx+1} ;
                    case 'verbose'
                        verbose = varargin{idx+1} ;
                    case 'plot_time_step'
                        plot_time_step = varargin{idx+1} ;
                end
            end
            
            if ~exist('sensor_radius','var')
                sensor_radius = 1 ;
            end
            
            if ~exist('position_indices','var')
                position_indices = [1 2] ;
            end
            
            if ~exist('n_states','var')
                n_states = 2 ;
            end
            
            if ~exist('n_inputs','var')
                n_inputs = 0 ;
            end
            
            if ~exist('orientation_indices','var')
                orientation_indices = [] ;
            end
            
            if ~exist('plot_sensor_radius','var')
                plot_sensor_radius = false ;
            end
            
            if ~exist('verbose','var')
                verbose = 0 ;
            end
            
            if ~exist('plot_time_step','var')
                plot_time_step = 1 ;
            end

            A.sensor_radius = sensor_radius ;
            A.plot_sensor_radius = plot_sensor_radius ;
            A.position_indices = position_indices ;
            A.orientation_indices = orientation_indices ;
            A.n_states = n_states ;
            A.n_inputs = n_inputs ;
            A.verbose = verbose ;
            A.plot_time_step = plot_time_step ;
        end
        
        %% reset
        function reset(A,position)
            % method: reset(position)
            %
            % Zeros out the agent's states and inputs, and sets its time to
            % zero. By default, the position input should be in SE2, i.e.
            % an (x,y,h) pose.
            
            position = position(:) ;
            A.state(A.position_indices) = position(1:2) ;
            if length(position) > 2
                A.state(A.orientation_indices) = position(3) ;
            end
            A.time = 0 ;
            A.input = zeros(A.n_inputs,1) ; % this forces the input size to
                                            % match the time vector size
            A.input_time = 0 ;
        end
        
        %% move
        function move(A,t_move,T_input,U_input,Z_desired)
            % method: move(t_move,T_input,U_input,Z_desired)
            %
            % Moves the agent according to the provided control input, and
            % attempting to track the provided trajectory. The assumption
            % is that the input is zero-order hold, and the input
            % corresponding to the last time index is zero; this is why
            % the last old input is discarded when the input is updated;
            % similarly, we assume that the time vector corresponding to
            % the input starts at 0.
            
            if T_input(end) < t_move
                warning(['Provided input time vector is shorter than the ',...
                        'desired motion time! The agent will only be ',...
                        'moved until the largest available time in the ',...
                        'input time vector.'])
                t_move = T_input(end) ;
            end
            
            
            T_input = T_input(:)' ;
            
            % if the inputs T, U, and Z correspond to a longer time horizon
            % than just t_move, truncate them
            tlog = T_input <= t_move ;
            T_used = T_input(tlog) ;
            if T_used(end) < t_move
                T_used = [T_used, t_move] ;
            end
            
            zcur = A.state(:,end) ;
            
            if nargin < 5 || isempty(Z_desired)
                U_used = matchTrajectories(T_used,T_input,U_input) ;
                [tout,zout] = A.ode_solver(@(t,z) A.dynamics(t,z,T_used,U_used,[]),...
                                          [0 t_move], zcur) ;
            else
                [U_used,Z_used] = matchTrajectories(T_used,T_input,U_input,T_input,Z_desired) ;
                [tout,zout] = A.ode_solver(@(t,z) A.dynamics(t,z,T_used,U_used,Z_used),...
                                      [0 t_move], zcur) ;
            end
            
            A.state = [A.state, zout(2:end,:)'] ;
            A.time = [A.time, A.time(end) + tout(2:end)'] ;
            A.input_time = [A.input_time, A.input_time(end) + T_used(2:end)] ;
            A.input = [A.input(:,1:end-1), U_used] ;
        end
        
        %% stop
        function stop(A,t)
        % method: stop(t)
        %
        % By default, calls the move function with zero input for duration
        % t seconds; should be overwritten in a subclass.
        
            A.move(t,[0 t], zeros(A.n_inputs,2)) ;
        end
        
        %% default dynamics
        function zd = dynamics(~,~,z,~,~)
        % dzdt = dynamics(t,z,T,U,Z)
        %
        % If z is the state, then this function returns the time derivative
        % of z. It takes in the current time t, state z, and time vector T
        % with the same length as the input vector U and desired trajectory
        % vector Z.
            zd = zeros(size(z)) ;
        end        
        
        %% default ODE solver
        function [tout,zout] = ode_solver(~,fun,tspan,z0)
            [tout,zout] = ode45(@(t,z) fun(t,z),tspan,z0(:)) ;
        end
        
        %% verbose display
        function vdisp(A,s,l)
        % Display a string s if the message's verbose level l is greater
        % than or equal to the planner's verbose level.
            if nargin < 3
                l = 1 ;
            end
            if A.verbose >= l
                if ischar(s)
                    disp(['    A: ',s])
                else
                    disp('    A: String not provided!')
                end
            end
        end
        
        %% plotting
        function plot(A,n,c)
            if nargin < 3
                c = [0 0 1] ;
                if nargin < 2
                    n = 1 ;
                end
            end
            
            figure(n)
            
            % plot trajectory
            xy = A.state(A.position_indices,:) ;
            plot(xy(1,:),xy(2,:),'Color',c)
            
            % plot sensor radius
            if A.plot_sensor_radius
                rsense = A.sensor_radius ;
                xcirc = rsense*cos(linspace(0,2*pi)) ;
                ycirc = rsense*sin(linspace(0,2*pi)) ;
                plot(xcirc + xy(1,end), ycirc + xy(2,end),...
                     '--','Color',[0.5 0.5 1])
            end
        end
    end
end