classdef agent2D < handle
% Class: agent2D
%
% Generic 2D agent with zero dynamics as default. Other agents should
% inherit this as a superclass.
%
% Note! This generic agent uses ode45 to forward-integrate its dynamics.
% Overwrite the 'odesolver' method in a subclass if you wish to use a
% different solver.
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
%   n_states             the number of states the agent has; typically the
%                        first two states are x and y coordinates, and the
%                        third state is the heading (e.g. SE2 coords)
%
%   n_inputs             the number of inputs the agent has
%
%   xy_state_indices     the indices of the x and y coordinates as a 1-by-2
%                        vector, by default [1 2]
%
%   heading_state_index  the index of the heading coordinate as a scalar,
%                        by default 3
%
%   footprint            the agent's "footprint" as a radius (scalar) or as
%                        length and width (1-by-2 vector); agents are
%                        either circular or rectangular, and by default
%                        circular with radius 1
%
%   footprint_contour    a counter-clockwise contour providing the
%                        footprint of the agent as a polyline for use in
%                        crash detection
%
%   sensor_radius        a scalar indicating the sensor radius of the
%                        agent, which can be used by the world to determine
%                        which obstacles and portions of the world to
%                        reveal to the agent
%
%   plot_sensor_radius   a boolean that determines whether or not to plot
%                        the sensor radius when plotInLoop is called
%
%   plot_time_step       a double that is used for plotting final results;
%                        number is the time step to use when plotting the
%                        agent's final trajectory
%
% METHODS:
%   agent2D              the constructor method, which any subclass of
%                        agent2D should typically call, since it
%                        establishes the default properties
%
%   reset                by default, sets the agent's state to the providedlog.world_info.goal
%                        position and its inputs and time to zero
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
%   odesolver            a solver used by the agent; by default ode45 is
%                        used, and should be sufficient for most agents;
%                        overwriting this in a subclass is useful for
%                        implementing RK4 or some other fixed-step
%                        integration method
%
%   plotInLoop           plots the agent's trajectory and footprint; this
%                        method is called once per iteration of the
%                        simulator

    properties
        state
        time
        input
        input_time
        n_states
        n_inputs
        xy_state_indices
        heading_state_index
        footprint
        footprint_contour
        sensor_radius
        verbose
        plot_sensor_radius
        plot_time_step
    end
    
    methods
    %% constructor
        function A = agent2D(varargin)
        % method: A = agent2D(verbose_level,varargin)
        %
        % This method creates an agent object. It takes in string/argument
        % pairs, where the strings are the names of the agent2D default
        % properties. Call "help agent2D" for a walkthrough of these
        % properties. Typically, a user will not call this method, but a
        % subclass of agent2D will call it in its own constructor method.
            if mod(length(varargin),2) == 1
                verbose_level = varargin{1} ;
                varargin = varargin(2:end) ;
            else
                verbose_level = 0 ;
            end
            A.verbose = verbose_level ;
       
            for idx = 1:2:length(varargin)
                switch varargin{idx}
                    case 'footprint'
                        footprint = varargin{idx+1} ;
                    case 'sensor_radius'
                        sensor_radius = varargin{idx+1} ;
                    case 'xy_state_indices'
                        xy_state_indices = varargin{idx+1} ;
                    case 'n_states'
                        n_states = varargin{idx+1} ;
                    case 'n_inputs'
                        n_inputs = varargin{idx+1} ;   
                    case 'heading_state_index'
                        heading_state_index = varargin{idx+1} ;
                    case 'plot_sensor_radius'
                        plot_sensor_radius = varargin{idx+1} ;
                    case 'verbose'
                        verbose = varargin{idx+1} ;
                    case 'plot_time_step'
                        plot_time_step = varargin{idx+1} ;
                end
            end
            
            if ~exist('footprint','var')
                footprint = 1 ;
            end
            
            if ~exist('sensor_radius','var')
                sensor_radius = 1 ;
            end
            
            if ~exist('xy_state_indices','var')
                xy_state_indices = [1 2] ;
            end
            
            if ~exist('n_states','var')
                n_states = 2 ;
            end
            
            if ~exist('n_inputs','var')
                n_inputs = 0 ;
            end
            
            if ~exist('heading_state_index','var')
                heading_state_index = [] ;
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
            
            A.footprint = footprint ;
            A.sensor_radius = sensor_radius ;
            A.plot_sensor_radius = plot_sensor_radius ;
            A.xy_state_indices = xy_state_indices ;
            A.heading_state_index = heading_state_index ;
            A.n_states = n_states ;
            A.n_inputs = n_inputs ;
            A.verbose = verbose ;
            A.plot_time_step = plot_time_step ;
            
            % create the robot footprint contour as either a circle or a
            % rectangle
            if length(footprint) == 1
                xc = footprint.*cos(linspace(0,2*pi)) ;
                yc = footprint.*sin(linspace(0,2*pi)) ;
                FC = [xc;yc] ;
            elseif length(footprint) == 2
                FC = 0.5.*[-1 1 1 -1 -1 ; -1 -1 1 1 -1].*repmat(footprint(:),1,5) ;
            else
                error(['The robot footprint must be given as either ',...
                       'a scalar (for a circular robot) or a 1-by-2 ',...
                       'vector (for a rectangular robot length and width).'])
            end
            A.footprint_contour = FC ;
        end
        
        %% reset
        function reset(A,position)
            % method: reset(position)
            %
            % Zeros out the agent's states and inputs, and sets its time to
            % zero. By default, the position input should be in SE2, i.e.
            % an (x,y,h) pose.
            
            position = position(:) ;
            A.state(A.xy_state_indices) = position(1:2) ;
            if length(position) > 2
                A.state(A.heading_state_index) = position(3) ;
            end
            A.time = 0 ;
            A.input = zeros(A.n_inputs,1) ; % this forces the input size to
                                            % match the time vector size
            A.input_time = 0 ;
        end
        
        %% move and predict
        function move(A,T_total,T_input,U_input,Z_desired)
            % method: move(T_total,T_input,U_input,Z_desired)
            %
            % Moves the agent according to the provided control input, and
            % attempting to track the provided trajectory. The assumption
            % is that the input is zero-order hold, and the input
            % corresponding to the last time index is zero; this is why
            % the last old input is discarded when the input is updated;
            % similarly, we assume that the time vector corresponding to
            % the input starts at 0.
            
            if T_input(end) < T_total
                warning(['Provided input time vector is shorter than the ',...
                        'desired motion time! The agent will only be ',...
                        'moved until the largest available time in the ',...
                        'input time vector.'])
                T_total = T_input(end) ;
            end
            
            
            T_input = T_input(:)' ;
            
            % if the inputs T, U, and Z correspond to a longer time horizon
            % than just T_total, truncate them
            tlog = T_input <= T_total ;
            T_used = T_input(tlog) ;
            if T_used(end) < T_total
                T_used = [T_used, T_total] ;
            end
            
            zcur = A.state(:,end) ;
            
            if nargin < 5 || isempty(Z_desired)
                U_used = matchTrajectories(T_used,T_input,U_input) ;
                [tout,zout] = A.odesolver(@(t,z) A.dynamics(t,z,T_used,U_used,[]),...
                                          [0 T_total], zcur) ;
            else
                [U_used,Z_used] = matchTrajectories(T_used,T_input,U_input,T_input,Z_desired) ;
                [tout,zout] = A.odesolver(@(t,z) A.dynamics(t,z,T_used,U_used,Z_used),...
                                      [0 T_total], zcur) ;
            end
            A.state = [A.state, zout(2:end,:)'] ;
            A.time = [A.time, A.time(end) + tout(2:end)'] ;
            A.input_time = [A.input_time, A.input_time(end) + T_used(2:end)] ;
            A.input = [A.input(:,1:end-1), U_used] ;
        end

        function prediction = predict(A,T_total,T_input,U_input,Z_desired,...
                                      tcur)
            % method: out = predict(T_total,T_input,U_input,Z_desired,tcur)
            %
            % See agent2D.move. This method performs the same calculation
            % as "move" but without updating the agent's state, time, or
            % input. This method also takes in an additional input, tcur,
            % which allows for a prediction from an arbitrary time (not
            % necessarily from the agent's most recent state, which is the
            % default initial condition for the prediction)
            
            if T_input(end) < T_total
                warning(['Provided input time vector is shorter than the ',...
                        'desired motion time! The robot will only be ',...
                        'moved until the largest available time in the ',...
                        'input time vector.'])
                T_total = T_input(end) ;
            end
            
            
            T_input = T_input(:)' ;
            
            % if "tcur" was provided, get the agent's state at or just
            % before tcur
            if nargin < 6
                zcur = A.state(:,end) ;
            else
                tlog = A.time <= tcur ;
                idxs = 1:length(A.time) ;
                idxs = idxs(tlog) ;
                zcur = A.state(:,idxs(end)) ;
            end
                
            
            if nargin < 5
                [tout,zout] = A.odesolver(@(t,z) A.dynamics(t,z,T_input,U_input,[]),...
                                          [0 T_total], zcur) ;
            else
                [tout,zout] = A.odesolver(@(t,z) A.dynamics(t,z,T_input,U_input,Z_desired),...
                                      [0 T_total], zcur) ;
            end
            prediction.state = [A.state, zout(2:end,:)'] ;
            prediction.time = [A.time, A.time(end) + tout(2:end)'] ;
            prediction.input_time = [A.input_time, A.input_time(end) + T_input(2:end)] ;
            prediction.input = [A.input(:,1:end-1), U_input] ;
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
        function [tout,zout] = odesolver(~,fun,tspan,z0)
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
        function plot(A)
            A.plotInLoop(1,[0 0 1])
            axis equal
        end
        
        function plotInLoop(A,n,c)
            if nargin < 3
                c = [0 0 1] ;
            end
            
            figure(n)
            
            % plot trajectory
            xy = A.state(A.xy_state_indices,:) ;
            plot(xy(1,:),xy(2,:),'Color',c)
            
            % plot robot footprint and sensor horizon
            rc = A.footprint_contour ;
            hidx = A.heading_state_index ;
            
            if ~isempty(hidx)
                % rotate the agent's footprint contour if needed
                h = A.state(hidx,end) ;
                R = rotmat(h) ;
                rc = R*rc ;
            end
            
            plot(rc(1,:) + xy(1,end), rc(2,:) + xy(2,end),'Color',c)
            
            if A.plot_sensor_radius
                rsense = A.sensor_radius ;
                xcirc = rsense*cos(linspace(0,2*pi)) ;
                ycirc = rsense*sin(linspace(0,2*pi)) ;
                plot(xcirc + xy(1,end), ycirc + xy(2,end),...
                     '--','Color',[0.5 0.5 1])
            end
        end
        
        function plotResults(A,n,c,s,i)
            figure(n) ;
            
            sidx = s(i) ;
            
            % plot trajectory
            z = sidx.trajectory ;
            plot(z(1,:),z(2,:),'Color',c)
            
            % plot agent along trajectory
            rc = A.footprint_contour ;
            hidx = A.heading_state_index ;
            T = A.time(end) - A.time(1) ;
            dt = A.plot_time_step ;
            Nplot = ceil(T/dt) ;
            tplotvec = linspace(A.time(1),A.time(end),Nplot) ;
            tsim = sidx.sim_time_vector ;
            zplot = matchTrajectories(tplotvec,tsim,z) ;
            for idx = 1:Nplot
                if ~isempty(hidx)
                    h = zplot(hidx,idx) ;
                    R = rotmat(h) ;
                    rcidx = R*rc + repmat(zplot(A.xy_state_indices,idx),1,size(rc,2)) ;
                else
                    rcidx = rc + repmat(zplot(A.xy_state_indices,idx),1,size(rc,2)) ;
                end
                
                plot(rcidx(1,:),rcidx(2,:),'Color',c)
            end
        end
        
        function plotFilledAgent(A,n,face_color,edge_color)
            if nargin <4
                edge_color = [0 0 1] ;
            if nargin <3
                face_color = [0.8 0.8 1] ;
            if nargin < 2
                n = 1 ;
            end
            end
            end
            figure(n) ;
            
            if ~isempty(A.heading_state_index)
                R = rotmat(A.state(A.heading_state_index,end)) ;
            else
                R = rotmat(0) ;
            end
            
            % plot footprint
            fp = R*A.footprint_contour + A.state(A.xy_state_indices,end) ;
            patch(fp(1,:),fp(2,:),face_color,'EdgeColor',edge_color,'LineWidth',1.5) ;
            
            % plot heading triangle
            if ~isempty(A.heading_state_index)
                t = linspace(0,2*pi,4) ;
                if length(A.footprint) == 2
                    D = max(A.footprint) / 2 ;
                    trans = mean([max(A.footprint_contour(1,:)),min(A.footprint_contour(1,:))]);
                    xdm=1/3;
                else
                    D = A.footprint ;
                    trans=0;
                    xdm=1/2;
                end

                triangle = R*[((D*xdm).*cos(t) +trans) ;...
                              (D/4).*sin(t)] ;

                triangle = triangle + A.state(A.xy_state_indices,end) ;

                patch(triangle(1,:),triangle(2,:),face_color,'EdgeColor',edge_color,'LineWidth',1.5) ;
            end
        end
    end
end