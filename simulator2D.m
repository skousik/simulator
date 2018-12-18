classdef simulator2D < handle
% Class: simulator2D
%
% S = simulator2D(agent, world, planners, varargin)
%
% This class is used to simulate an agent (i.e. a robot in SE2 with a
% footprint and system dynamics) in a world (given by polygonal obstacles
% and a bounding box). The agent plans its path through the world from some
% start location to some end goal using one or more path planners. This
% simulator is set up to allow for the testing of multiple path planning
% methods on randomly-generated worlds.
%
% For simulator2D to function correctly, the agent, world, and planners
% must have certain methods and properties defined. More details for each
% can be found in the superclasses: agent2D, world2D, and planner 2D.
%
% The simulator itself has several useful properties and methods.
% Properties:
%   stop_count - the number of times that a planner has failed to plan
%                consecutively
%
%   stop_threshold - if stop_count is greater than 
%   stop_sim_when_crashed
%   allow_replan_errors
%   save_planner_info

%% properties
    properties (Access = public)
        % basic properties
        agent
        world
        planners
        verbose
        
        % simulation
        stop_count
        stop_threshold
        stop_sim_when_crashed
        allow_replan_errors
        save_planner_info
        manual_iteration
        
        % plotting
        figure_number
        figure_handle
        planner_colors
        plotting_pause_time
        plot_order
        save_gif
        save_gif_filename
        save_gif_delay_time
    end
    
    properties (Access = private)
        start_gif
    end
    
%% methods
    methods
    %% constructors
        function S = simulator2D(agent, world, planners, varargin)
            % Constructor function: simulator2D
            %
            % Usage: S = simulator2D(agent, world, planners, varargin)
            %
            % This constructor takes in an agent, which obeys some physical
            % dynamics; a world object, which contains obstacles and goals;
            % and one or more planners for the agent to perform path
            % planning and obstacle avoidance in the world. The constructor
            % then sets up internal variables needed to simulate the agent
            % in the provided world using each of the provided planners.
            
            if nargin < 4
                verbose_level = 0 ;
            if nargin < 3
                planners = planner2D ;
            if nargin < 2
                world = world2D ;
            if nargin < 1
                agent = agent2D ;
            end
            end
            end
            end
            
            if length(varargin) == 1
                verbose_level = varargin{1} ;
            else
                for idx = 1:2:length(varargin)
                    switch varargin{idx}
                        case 'stop_threshold'
                            stop_threshold = varargin{idx+1} ;
                        case 'figure_number'
                            figure_number = varargin{idx+1} ;
                        case 'verbose_level'
                            verbose_level = varargin{idx+1} ;
                        case 'stop_sim_when_crashed'
                            stop_sim_when_crashed = varargin{idx+1} ;
                        case 'allow_replan_errors'
                            allow_replan_errors = varargin{idx+1} ;
                        case 'save_planner_info'
                            save_planner_info = varargin{idx+1} ;
                        case 'planner_colors'
                            planner_colors = varargin{idx+1} ;
                        case 'manual_iteration'
                            manual_iteration = varargin{idx+1} ;
                        case 'plotting_pause_time'
                            plotting_pause_time = varargin{idx+1} ;
                        case 'plot_order'
                            plot_order = varargin{idx+1} ;
                        case 'save_gif'
                            save_gif = varargin{idx+1} ;
                        case 'save_gif_filename'
                            save_gif_filename = varargin{idx+1} ;
                        case 'save_gif_delay_time'
                            save_gif_delay_time = varargin{idx+1} ;
                        otherwise
                            error(['Keyword argument number ',...
                                   num2str((idx+1)/2),' is invalid!'])
                    end
                end
            end
            
            if ~exist('stop_threshold','var')
                stop_threshold = 5 ;
            end
            
            if ~exist('figure_number','var')
                figure_number = 1 ;
            end
            
            if ~exist('verbose_level','var')
                verbose_level = 0 ;
            end
            
            if ~exist('stop_sim_when_crashed','var')
                stop_sim_when_crashed = true ;
            end
            
            if ~exist('allow_replan_errors','var')
                allow_replan_errors = false ;
            end
            
            if ~exist('save_planner_info','var')
                save_planner_info = true ;
            end
            
            if ~exist('planner_colors','var')
                planner_colors = repmat([0 0 1],length(planners),1) ;
            end
            
            if ~exist('manual_iteration','var')
                manual_iteration = false ;
            end
            
            if ~exist('plotting_pause_time','var')
                plotting_pause_time = 0.2 ;
            end
            
            if ~exist('plot_order','var')
                plot_order = 'WAP' ;
            end
            
            if ~exist('save_gif','var')
                save_gif = false ;
            end
            
            if ~exist('save_gif_filename','var')
                save_gif_filename = 'simulator2D_gif_output' ;
            end
            
            if ~exist('save_gif_delay_time','var')
                save_gif_delay_time = 0.1 ;
            end
            
            if ~iscell(planners) && length(planners) == 1
                planners = {planners} ;
            end
            
            S.agent = agent ;
            S.world = world ;
            S.planners = planners ;
            
            % set the world to the same verbosity level as the simulator
            if S.world.verbose < verbose_level
                S.world.verbose = verbose_level ;
            end
            
            S.verbose = verbose_level ;
            S.figure_number = figure_number ;
            S.stop_threshold = stop_threshold ;
            S.stop_sim_when_crashed = stop_sim_when_crashed ;
            S.allow_replan_errors = allow_replan_errors ;
            S.save_planner_info = save_planner_info ;
            S.manual_iteration = manual_iteration ;
            S.planner_colors = planner_colors ;
            S.plotting_pause_time = plotting_pause_time ;
            S.plot_order = plot_order ;
            S.save_gif = save_gif ;
            S.save_gif_filename = save_gif_filename ;
            S.save_gif_delay_time = save_gif_delay_time ;
            S.start_gif = save_gif ;
        end
        
        function plannerSetup(S,planner_indices)
            % Method: plannerSetup
            %
            % This function runs the setup function/s of the planner/s
            % indicated by the provided planner indices. Since the setup
            % function can take a large amount of time, ideally this
            % function is only called once.
            S.vdisp('Running planner setup')
            
            if ~exist('planner_indices','var')
                planner_indices = 1:length(S.planners) ;
            end

            for p = planner_indices
                S.vdisp(['Setting up planner ',num2str(p)],2)
                S.planners{p}.setup(S.agent,S.world) ;
            end
        end
        
        function plannerUpdate(S,planner_indices)
            % Method: plannerUpdate
            %
            % This function runs the update function/s of the planner/s
            % indicated by the provided planner indices. Since the world
            % may change from trial to trial (for example by generating new
            % obstacles or start/goal points), the planners need to be
            % updated
            S.vdisp('Updating planners')
            
            if ~exist('planner_indices','var')
                planner_indices = 1:length(S.planners) ;
            end
            
            for p = planner_indices
                S.vdisp(['Updating planner ',num2str(p)],2)
                S.planners{p}.update(S.agent,S.world) ;
            end
        end
        
    %% simulate
        function summary = runSimulation(S,t_max,iter_max,...
                                         planner_indices,plot_in_loop)
            % Method: runSimulation
            %
            % This function simulates the agent in the provided world as it
            % attempts to reach the goal from its provided start position.
            S.vdisp('Running simulation')
            
            if ~exist('planner_indices','var')
                planner_indices = 1:length(S.planners) ;
            end
            
            if ~exist('plot_in_loop','var')
                plot_in_loop = false ;
            end
            
            A = S.agent ;            
            W = S.world ;
            
            % set up output objects
            L = length(planner_indices) ;
            planner_name = cell(1,L) ;
            planner_info = cell(1,L) ;
            trajectory = cell(1,L) ;
            total_real_time = cell(1,L) ;
            planning_times = cell(1,L) ;
            crash_check = cell(1,L) ;
            goal_check = cell(1,L) ;
            stop_check = cell(1,L) ;
            sim_time = cell(1,L) ;
            control_input = cell(1,L) ;
            control_input_time = cell(1,L) ;
            planner_timeout = cell(1,L) ;
            
            % index for keeping track of which planner was used
            idx = 1 ;
            
        %% planner loop (simulated scenario is run once for each planner)
            for p = planner_indices
                S.vdisp(['Planner ',num2str(p)])
                
                P = S.planners{p} ;
                A.reset(W.start) ;
                W.reset(P) ;
                
                planning_time_vec = nan(1,iter_max) ;
                
                % reset the stop counter
                S.stop_count = 0 ;
                stop_check_vec = false(1,iter_max) ;

                runtime = tic ;
                
                icur = 1 ;
                tstart = tic ;
                tcur = toc(tstart);
                
            %% simulation loop
                while icur < (iter_max+1) && tcur < t_max
                    S.vdisp('--------------------------------',3,false)
                    S.vdisp(['ITERATION ',num2str(icur)],2,false)
                    
                %% sense world
                    % given the current state of the agent, query the world
                    % to get the surrounding obstacles
                    O = W.getNearbyObstacles(A,P) ;
                    
                %% replan
                    % given the current state and obstacles, query the
                    % current planner to get a control input
                    t_plan_spent = tic ;
                    if S.allow_replan_errors
                        [T,U,Z_plan] = P.replan(A,O) ;
                    else
                        try
                            [T,U,Z_plan] = P.replan(A,O) ;
                        catch
                            S.vdisp(['Planner ',num2str(p),' errored while ',...
                                     'replanning!'])
                            T = [] ; U = [] ; Z_plan = [] ;
                        end
                    end
                    t_plan_spent = toc(t_plan_spent) ;
                    planning_time_vec(icur) = t_plan_spent ;
                    S.vdisp(['Planning time: ',num2str(t_plan_spent),' s'],4)
                    
                %% move agent
                    % update the agent using the current control input, so
                    % either stop if no control was returned, or move the
                    % agent if a valid input and time vector were returned
                    if size(T,2) < 2 || size(U,2) < 2 || T(end) == 0
                        S.vdisp('Stopping!',2)
                        A.stop() ;
                        
                        stop_check_vec(icur) = true ;
                        
                        % give planner a chance to recover from a stop
                        S.stop_count = S.stop_count + 1 ;
                        if S.stop_count > S.stop_threshold
                            break
                        end
                    else
                        S.stop_count = 0 ;
                        if P.t_move > T(end)
                            S.vdisp(['The provided time vector for the ',...
                                'agent input is shorter than the amount of ',...
                                'time the agent must move at each ',...
                                'planning iteration. The agent will only ',...
                                'be moved for the duration of the ',...
                                'provided time vector.'],3)
                            t_move = T(end) ;
                        else
                            t_move = P.t_move ;
                        end
                        
                        A.move(t_move,T,U,Z_plan) ;
                    end
                
                %% Note (9 Nov 2018)
                % For now, dynamic obstacles are treated as follows:
                %   1) getNearbyObstacles should return a prediction
                %   2) the agent is moved according to the prediction
                %   3) crashCheck moves the obstacles (according to the
                %      agent's movement data if needed)
                    
                %% crash and goal check
                    % check if the agent is near the desired goal or if it
                    % crashed
                    S.vdisp('Checking if agent reached goal or crashed...',3)
                    goalCheck = W.goalCheck(A) ;
                    crashCheck = W.crashCheck(A, false) ;
                    
                    if crashCheck && S.stop_sim_when_crashed
                        S.vdisp('Crashed!',2) ;
                        break
                    end
                    
                    if goalCheck
                        S.vdisp('Reached goal!',2) ;
                        break
                    end
                    
                    % plotting and animation
                    if plot_in_loop
                        S.plotInLoop(p)
                        if S.save_gif
                            
                        else
                            pause(S.plotting_pause_time) ;
                        end
                    end
                    
                    % pause for user if needed
                    if S.manual_iteration
                        user_pause = tic ;
                        S.vdisp('Pausing for user. Press any key to continue.',2)
                        pause
                        user_pause = toc(user_pause) ;
                    else
                        user_pause = 0 ;
                    end
                    
                    % iterate and increment time
                    S.vdisp(['END ITERATION ',num2str(icur)],4,false)
                    icur = icur + 1 ;
                    tcur = toc(tstart) - user_pause ;
                end
                runtime = toc(runtime) ;
                S.vdisp(['Planning time spent: ',num2str(runtime)],5)
                
                % plot the last portion of the agent's trajectory after the
                % simulation ends
                if plot_in_loop
                    S.plotInLoop(p)
                end
                
                S.vdisp(['Planner ',num2str(p), ' simulation complete!'])
                
            %% create summary (for the current planner)
                % get results at end of simulation
                S.vdisp('Running final crash and goal checks.',2)
                Z = A.state ;
                T = A.time ;
                U = A.input ;
                TU = A.input_time ;
                C = W.crashCheck(A) ;
                G = W.goalCheck(A) ;
                
                if S.save_planner_info
                    planner_info{idx} = S.planners{p}.info ;
                else
                    planner_info{idx} = 'no info saved' ;
                end
                
                % fill in the results for the current planner
                planner_name{idx} = P.name ;
                trajectory{idx} = Z ;
                sim_time{idx} = T ;
                control_input{idx} = U ;
                control_input_time{idx} = TU ;
                total_real_time{idx} = runtime ;
                planning_times{idx} = planning_time_vec ;
                crash_check{idx} = C ;
                goal_check{idx} = G ;
                stop_check{idx} = stop_check_vec ;
                planner_timeout{idx} = P.timeout ;
                
                if G
                    S.vdisp('In final check, agent reached goal!')
                end
                if C
                    S.vdisp('In final check, agent crashed!')
                end    
                
                % increment planner index
                idx = idx + 1 ;
            end
            
            S.vdisp('Simulation complete! Generating summary.')
            
            summary = struct('planner_name',planner_name,...
                             'trajectory',trajectory,...
                             'total_real_time',total_real_time,...
                             'planning_time',planning_times,...
                             'crash_check',crash_check,...
                             'goal_check',goal_check,...
                             'stop_check',stop_check,...
                             'sim_time_vector',sim_time,...
                             'control_input',control_input,...
                             'control_input_time',control_input_time,...
                             'planner_indices',planner_indices,...
                             'obstacles',W.obstacles,...
                             'N_obstacles',W.N_obstacles,...
                             't_max',t_max,...
                             'planner_timeout',planner_timeout,...
                             't_move',P.t_move,...
                             'iter_max',iter_max,...
                             'start',W.start,...
                             'goal',W.goal,...
                             'bounds',W.bounds,...
                             'notes','',...
                             'planner_info',planner_info) ;
        end
        
        %% plotting
        function plotInLoop(S,planner_index)
            S.vdisp('Plotting in loop',3)
            
            fh = figure(S.figure_number) ;
            cla ; hold on ; axis equal ;
            
            if ~any(isinf(S.world.bounds))
                axis(S.world.bounds)
            end
            
            color = S.planner_colors(planner_index,:) ;
            
            for plot_idx = S.plot_order
                switch plot_idx
                    case 'W'
                        S.world.plotInLoop(S.figure_number)
                    case 'A'
                        S.agent.plotInLoop(S.figure_number,color)  
                    case 'P'
                        S.planners{planner_index}.plotInLoop(S.figure_number,color)
                    otherwise
                        error(['Simulator plot order is broken! Make sure ',...
                              'it is a string containing the characters ',...
                              'W (for world), A (for agent), and P (for ',...
                              'planner), in the order you want them to ',...
                              'plot (WAP is the default)'])
                end
            end
            
            if S.save_gif
                if S.start_gif
                    % if gif saving is enabled, check that there isn't already a
                    % file in the current directory with that name
                    dir_content = dir(pwd) ;
                    file_name   = {dir_content.name} ;
                    check_name = [S.save_gif_filename,'_planner_',num2str(planner_index),'.gif'] ;
                    file_check  = any(cellfun(@(x) strcmp(check_name,x),file_name)) ;
                    if file_check
                        warning(['The current GIF filename already exists! ',...
                            'change the filename if you do not want to ',...
                            'overwrite your existing file!'])
                    end
                    
                    S.vdisp(['Please resize the figure to the size you ',...
                             'want saved, or hit any key to continue.'])
                    pause
                end
                
                frame = getframe(fh) ;
                im = frame2im(frame);
                [imind,cm] = rgb2ind(im,256);
                
                filename = [S.save_gif_filename,'_planner_',num2str(planner_index),'.gif'] ;

                if S.start_gif
                    imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',S.save_gif_delay_time) ; 
                    S.start_gif = false ;
                else 
                    imwrite(imind,cm,filename,'gif','WriteMode','append') ; 
                end
            end
        end
        
        function plotResults(S,summary)
            S.vdisp('Plotting simulation results',2)
            
            figure(S.figure_number+1) ;
            cla ; hold on ; axis equal ;
            
            S.world.plotResults(S.figure_number+1) ;
            
            for summary_index = 1:length(summary)
                planner_index = summary(1).planner_indices(summary_index) ;
                color = S.planner_colors(planner_index,:) ;
                S.agent.plotResults(S.figure_number+1,color,summary,planner_index) ;
                S.planners{planner_index}.plotResults(S.figure_number+1,color,summary) ;
            end
            
            axis(S.world.bounds)
        end
        
        %% verbose text output
        function vdisp(S,s,l,use_header)
        % Display a string 's' if the verbosity is greater than or equal to
        % the level 'l'; by default, the level is set to 0 and the default
        % threshold for displaying a message is 1 (so messages do not
        % display by default)
            if nargin < 4
                use_header = true ;
                if nargin < 3
                    l = 1 ; 
                end
            end
            
            if S.verbose >= l
                if use_header
                    disp(['S: ',s])
                else
                    disp(s)
                end
            end
        end
        
    end
end
