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
%                        input timing given by the A.input_time property
%
%   input_time           1-by-M vector indicating the timing of the input
%
%   n_states             the number of states the agent has; 2 by default
%
%   n_inputs             the number of inputs the agent has; 0 by default
%
%   position_indices     indices of the position coordinates as a 1-by-2
%                        vector for 2-D or 1-by-3 for 3-D; by default, this
%                        is [1,2]
%
%   orientation_indices  indices of yaw (for 2-D) and pitch and roll (3-D);
%                        by default empty
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
%   stop                 by default, calls "move" with zero control inputs
%                        for 1 second; should be replaced in a subclass
%                        with a "braking" functionality, typically
%                        implemented by calling A.move
%
%   dynamics             the differential equation describing the agent's
%                        motion given an input, input timing, and
%                        optionally a reference trajectory; by default
%                        zero, and should be overwritten in a subclass
%
%   integrator           a solver used by the agent; by default ode45 is
%                        used, and should be sufficient for most agents;
%                        overwriting this in a subclass is useful for
%                        implementing RK4 or some other fixed-step
%                        integration method
%
%   plot                 plots the agent's trajectory and footprint; this
%                        method is called once per iteration of the
%                        simulator

    properties
        state = [] ;
        time = [] ;
        input = [] ;
        input_time = [] ;
        n_states = 2 ;
        n_inputs = 0 ;
        position_indices = [1 2] ;
        sensor_radius = 1 ;
        verbose = 0 ;
        plot_data = struct('trajectory',[],'sensor_radius',[]) ;
        plot_sensor_radius = false
        plot_time_step = false ;
        LLC = [] ; % low-level controller
    end
    
    methods
    %% constructor
        function A = agent(varargin)
        % method: A = agent(verbose_level,'property',value,...)
        %
        % This method creates an agent object. It takes in string/argument
        % pairs, where the strings are the names of the agent default
        % properties.
            if mod(length(varargin),2) == 1
                verbose_level = varargin{1} ;
                varargin = varargin(2:end) ;
                A.verbose = verbose_level ;
            end

            for idx = 1:2:length(varargin)
                A.(varargin{idx}) = varargin{idx+1} ;
            end
            
            A.reset() ;
        end
        
        %% reset
        function reset(A,state)
            % method: reset(state)
            %
            % Zeros out the agent's states and inputs, and sets its time to
            % zero. The state input can be in R^2, i.e. a point (x,y), or
            % it can be A.n_states-by-1.
            
            if nargin < 2
                state = zeros(A.n_states,1) ;
            end
            
            A.state = zeros(A.n_states,1) ;
            if length(state) < A.n_states
                A.state(A.position_indices) = state ;
            else
                A.state = state(:) ;
            end
            A.time = 0 ;
            A.input = zeros(A.n_inputs,1) ; % this forces the input size to
                                            % match the time vector size
            A.input_time = 0 ;
        end
        
        %% move
        function move(A,t_move,T_ref,U_ref,Z_ref)
            % method: move(t_move,T_ref,U_ref,Z_ref)
            %
            % Moves the agent for the duration t_move using the nominal
            % inputs U_ref and nominal trajectory Z_ref that are indexed by
            % the nominal time T_ref.
            %
            % This method assumes that the input is zero-order hold, and
            % the input corresponding to the last time index is zero; this
            % is why the last old input is discarded when the input is
            % updated. Similarly, this method assumes that the nominal time
            % starts at 0.
            
            % get the time, input, and reference trajectory to use for
            % moving the agent
            [T_used,U_used,Z_used] = A.move_setup(t_move,T_ref,U_ref,Z_ref) ;
            
            % get the current state
            zcur = A.state(:,end) ;
            
            % call the ode solver to simulate agent
            [tout,zout] = A.integrator(@(t,z) A.dynamics(t,z,T_used,U_used,Z_used),...
                                       [0 t_move], zcur) ;
            
            A.commit_move_data(tout,zout,T_used,U_used,Z_used) ;
        end
        
        function [T,U,Z] = move_setup(A,t_move,T_ref,U_ref,Z_ref)
            % method: [T,U,Z] = move_setup(A,t_move,T_ref,U_ref,Z_ref)
            %
            % Given an amount of time t_move for the agent to move, and a
            % reference time vector, output a time vector to actually use
            % during the call to move the agent.
            
            % timing setup
            if T_ref(end) < t_move
                A.vdisp(['Provided input time vector is shorter than the ',...
                        'desired motion time! The agent will only be ',...
                        'moved until the largest available time in the ',...
                        'input time vector.'])
                t_move = T_ref(end) ;
            end

            T_ref = T_ref(:)' ;
            
            tlog = T_ref <= t_move ;
            T = T_ref(tlog) ;
            if T(end) < t_move
                T = [T, t_move] ;
            end
            
            % input and reference traj setup
            if nargin < 5 || isempty(Z_ref)
                U = match_trajectories(T,T_ref,U_ref) ;
                Z = [] ;
            else
                [U,Z] = match_trajectories(T,T_ref,U_ref,T_ref,Z_ref) ;
            end
        end
        
        function commit_move_data(A,T_state,Z_state,T_used,U_used,~)
            % method: commit_move_data(T_state,Z_state,T_input,U_input,Z_input)
            %
            % After moving the agent, commit the new state and input
            % trajectories, and associated time vectors, to the agent's
            % state, time, input, and input_time properties.
            
            A.state = [A.state, Z_state(:,2:end)] ;
            A.time = [A.time, A.time(end) + T_state(2:end)] ;
            A.input_time = [A.input_time, A.input_time(end) + T_used(2:end)] ;
            A.input = [A.input, U_used(:,1:end-1)] ;
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
        
        %% default integrator
        function [tout,zout] = integrator(~,fun,tspan,z0)
            [tout,zout] = ode45(@(t,z) fun(t,z),tspan,z0(:)) ;
            tout = tout' ;
            zout = zout' ;
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
        function plot(A,color)
            if nargin < 2
                color = [0 0 1] ;
            end
            
            % set up data to plot
            xy = A.state(A.position_indices,:) ;
            rsense = A.sensor_radius ;
            xcirc = rsense*cos(linspace(0,2*pi)) + xy(1,end) ;
            ycirc = rsense*sin(linspace(0,2*pi)) + xy(2,end) ;
            
            % check if a figure is up; if so, create a new figure,
            % otherwise update the existing data
            if check_if_plot_is_available(A,'trajectory')
                A.plot_data.trajectory.XData = xy(1,:) ;
                A.plot_data.trajectory.YData = xy(2,:) ;
                
                if A.plot_sensor_radius
                    A.plot_data.sensor_radius.XData = xcirc ;
                    A.plot_data.sensor_radius.YData = ycirc ;
                end
            else
                % plot trajectory
                trajectory_data = plot(xy(1,:),xy(2,:),'Color',color) ;
                A.plot_data.trajectory = trajectory_data ;

                % plot sensor radius
                if A.plot_sensor_radius
                    hold on
                    sensor_radius_data = plot(xcirc, ycirc,'--','Color',[0.5 0.5 1]) ;
                    A.plot_data.sensor_radius = sensor_radius_data ;
                    hold off
                end
            end
        end
    end
end