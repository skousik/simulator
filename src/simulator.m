classdef simulator < handle
% Class: simulator
%
% S = simulator(agent, worlds, planners, varargin)

%% properties
    properties (Access = public)
        % basic properties
        agent = agent() ;
        worlds = {world()} ;
        planners = {planner()} ;
        verbose = 1 ;

        % simulation
        max_sim_time = 100 ; % s per planner
        max_sim_iterations = 100 ; % per planner
        stop_count = 0 ;
        stop_threshold = 20 ;
        stop_sim_when_crashed = true ;
        allow_replan_errors = false ;
        save_planner_info = false ;
        manual_iteration = false ;
        run_full_collision_check_at_end = false ;

        % plotting
        figure_number = 1 ;
        figure_handle = [] ;
        planner_colors = [] ;
        plot_while_running = true ;
        plotting_pause_time = 0.1 ; % s
        plot_order = 'WAP' ;
        save_gif = false ;
        save_gif_filename = 'simulator_gif_output.gif' ;
        save_gif_delay_time = 0.1 ;
        start_gif = true ;
        manually_resize_gif = true ;
        animation_time_discretization = 0.1 ; % s
    end

%% methods
    methods
    %% constructor
        function S = simulator(agent, worlds, planners, varargin)
            % Constructor function: simulator
            %
            % Usage: S = simulator(agent, world, planners, varargin)
            %
            % This constructor takes in an agent, which obeys some physical
            % dynamics; a world object, which contains obstacles and goals;
            % and one or more planners for the agent to perform path
            % planning and obstacle avoidance in the world. The constructor
            % then sets up internal variables needed to simulate the agent
            % in the provided world using each of the provided planners.

            % parse inputs
            S = parse_args(S,varargin{:}) ;

            % if world or planner is alone, wrap it in a cell
            if length(worlds) == 1 && ~iscell(worlds)
                worlds = {worlds} ;
            end

            if length(planners) == 1 && ~iscell(planners)
                planners = {planners} ;
            end

            % check for planner colors
            if isempty(S.planner_colors)
                S.planner_colors = repmat([0 0 1],length(planners),1) ;
            end

            % wrap up construction
            S.agent = agent ;
            S.worlds = worlds ;
            S.planners = planners ;
        end

    %% run simulation
        function summary = run(S)
            % Method: run
            %
            % This function simulates the agent in the provided world as it
            % attempts to reach the goal from its provided start position.
            S.vdisp('Running simulation')

            % get simulation info
            t_max = S.max_sim_time ;
            iter_max = S.max_sim_iterations ;
            plot_in_loop_flag = S.plot_while_running ;

            % get world and planner indices
            world_indices = 1:length(S.worlds) ;
            planner_indices = 1:length(S.planners) ;

            % get agent
            A = S.agent ;

            % set up summaries
            LW = length(world_indices) ;
            summary = cell(1,LW) ;

        %% world loop
            for widx = world_indices
                W = S.worlds{widx} ;

                % set up summary objects
                L = length(planner_indices) ;
                planner_name = cell(1,L) ;
                planner_info_cell = cell(1,L) ;
                agent_info_cell = cell(1,L) ;
                trajectory = cell(1,L) ;
                total_real_time = cell(1,L) ;
                planning_times_cell = cell(1,L) ;
                collision_check_cell = cell(1,L) ;
                goal_check_cell = cell(1,L) ;
                stop_check_cell = cell(1,L) ;
                sim_time = cell(1,L) ;
                control_input = cell(1,L) ;
                control_input_time = cell(1,L) ;
                planner_timeout = cell(1,L) ;
                obstacles = cell(1,L);
            %% planner loop
                for pidx = planner_indices
                    S.vdisp(['Planner ',num2str(pidx)])

                    % get the planner
                    P = S.planners{pidx} ;

                    % get agent and world ready
                    W.reset() ;
                    A.reset(W.start) ;

                    % get planner ready
                    agent_info = A.get_agent_info() ;
                    world_info = W.get_world_info(agent_info,P) ;

                    P.setup(agent_info,world_info) ;

                    % check to make sure gif start is ready
                    if S.save_gif
                        S.start_gif = true ;
                    end

                    % initial plot
                    if plot_in_loop_flag
                        S.plot(widx,pidx)
                    end

                    % preallocate for storing planning time spent
                    planning_time_vec = nan(1,iter_max) ;

                    % reset the stop counter
                    S.stop_count = 0 ;
                    stop_check_vec = false(1,iter_max) ;

                    % start timing
                    icur = 1 ;
                    runtime = tic ;
                    tstart = runtime ;
                    tcur = toc(tstart);

                %% simulation loop
                    while icur < (iter_max+1) && tcur < t_max
                        S.vdisp('--------------------------------',3,false)
                        S.vdisp(['ITERATION ',num2str(icur),' (t = ',num2str(A.time(end)),')'],2,false)

                    %% get agent info
                        agent_info = A.get_agent_info() ;

                    %% get world info
                        % given the current state of the agent, query the world
                        % to get the surrounding obstacles
                        world_info = W.get_world_info(agent_info,P) ;

                    %% replan
                        % given the current state and obstacles, query the
                        % current planner to get a control input
                        t_plan_spent = tic ;
                        if S.allow_replan_errors
                            [T_nom,U_nom,Z_nom] = P.replan(agent_info,world_info) ;
                        else
                            try
                                [T_nom,U_nom,Z_nom] = P.replan(agent_info,world_info) ;
                            catch
                                S.vdisp(['Planner ',num2str(pidx),' errored while ',...
                                         'replanning!'])
                                T_nom = [] ; U_nom = [] ; Z_nom = [] ;
                            end
                        end
                        t_plan_spent = toc(t_plan_spent) ;
                        planning_time_vec(icur) = t_plan_spent ;
                        S.vdisp(['Planning time: ',num2str(t_plan_spent),' s'],4)

                    %% move agent
                        % update the agent using the current control input, so
                        % either stop if no control was returned, or move the
                        % agent if a valid input and time vector were returned
                        if size(T_nom,2) < 2 || size(U_nom,2) < 2 || T_nom(end) == 0
                            S.vdisp('Stopping!',2)
                            A.stop(P.t_move) ;

                            stop_check_vec(icur) = true ;

                            % give planner a chance to recover from a stop
                            S.stop_count = S.stop_count + 1 ;
                            if S.stop_count > S.stop_threshold
                                break
                            end
                        else
                            S.stop_count = 0 ;

                            if ~isempty(P.t_move)
                                if P.t_move > T_nom(end)
                                    S.vdisp(['The provided time vector for the ',...
                                        'agent input is shorter than the amount of ',...
                                        'time the agent must move at each ',...
                                        'planning iteration. The agent will only ',...
                                        'be moved for the duration of the ',...
                                        'provided time vector.'],3)

                                    t_move = T_nom(end) ;
                                else
                                    t_move = P.t_move ;
                                end
                            else
                                error(['Planner ',num2str(pidx),...
                                       '''s t_move property is empty!'])
                            end

                            A.move(t_move,T_nom,U_nom,Z_nom) ;
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
                        agent_info = A.get_agent_info() ;
                        goal_check = W.goal_check(agent_info) ;
                        collision_check = W.collision_check(agent_info,false) ;

                        if collision_check && S.stop_sim_when_crashed
                            S.vdisp('Crashed!',2) ;
                            break
                        end

                        if goal_check
                            S.vdisp('Reached goal!',2) ;
                            break
                        end

                        % plotting and animation
                        if plot_in_loop_flag
                            S.plot(widx,pidx)
                            if S.save_gif
                                error('Shreyas this is unfinished!')
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
                    if plot_in_loop_flag
                        S.plot(widx,pidx)
                    end

                    S.vdisp(['Planner ',num2str(pidx), ' simulation complete!'])

                %% create summary (for the current planner)
                    % get results at end of simulation
                    S.vdisp('Compiling summary',3)
                    Z = A.state ;
                    T_nom = A.time ;
                    U_nom = A.input ;
                    TU = A.input_time ;
                    agent_info = A.get_agent_info() ;
                    
                    if S.run_full_collision_check_at_end
                        S.vdisp('Running final collision check.',4)
                        collision_check = W.collision_check(agent_info) ;
                    end
                    
                    S.vdisp('Running final goal check',4)
                    goal_check = W.goal_check(agent_info) ;
                    agent_info_cell{pidx} = agent_info ;

                    if S.save_planner_info
                        planner_info_cell{pidx} = S.planners{pidx}.info ;
                    else
                        planner_info_cell{pidx} = 'no info saved' ;
                    end

                    % fill in the results for the current planner
                    planner_name{pidx} = P.name ;
                    trajectory{pidx} = Z ;
                    sim_time{pidx} = T_nom ;
                    control_input{pidx} = U_nom ;
                    control_input_time{pidx} = TU ;
                    total_real_time{pidx} = runtime ;
                    planning_times_cell{pidx} = planning_time_vec ;
                    collision_check_cell{pidx} = collision_check ;
                    goal_check_cell{pidx} = goal_check ;
                    stop_check_cell{pidx} = stop_check_vec ;
                    planner_timeout{pidx} = P.timeout ;
                    obstacles{pidx} = W.obstacles;
                    if goal_check
                        S.vdisp('In final check, agent reached goal!')
                    end
                    if collision_check
                        S.vdisp('In final check, agent crashed!')
                    end
                end
                S.vdisp(['World ',num2str(widx),' complete! Generating summary.'])
                summary{widx} = struct('planner_name',planner_name,...
                                 'trajectory',trajectory,...
                                 'total_real_time',total_real_time,...
                                 'total_iterations',icur,...
                                 'planning_time',planning_times_cell,...
                                 'crash_check',collision_check,...
                                 'goal_check',goal_check_cell,...
                                 'stop_check',stop_check_cell,...
                                 'sim_time_vector',sim_time,...
                                 'control_input',control_input,...
                                 'control_input_time',control_input_time,...
                                 'planner_indices',planner_indices,...
                                 'obstacles',obstacles,...
                                 'N_obstacles',W.N_obstacles,...
                                 't_max',t_max,...
                                 'planner_timeout',planner_timeout,...
                                 't_move',P.t_move,...
                                 'iter_max',iter_max,...
                                 'start',W.start,...
                                 'goal',W.goal,...
                                 'bounds',W.bounds,...
                                 'notes','',...
                                 'agent_info',agent_info_cell,...    
                                 'planner_info',planner_info_cell) ;
            end

            % clean up summary if only one world was run
            if LW == 1
                summary = summary{1} ;
            end

            S.vdisp('Simulation complete!')


        end

    %% plotting
    function plot(S,world_index,planner_index)
        S.vdisp('Plotting',3)

        if nargin < 3
            planner_index = 1 ;
            S.vdisp('Plotting planner 1 by default!',1) ;
            if nargin < 2
                world_index = 1 ;
                S.vdisp('Plotting world 1 by default!',1) ;
            end
        end

        fh = figure(S.figure_number) ;
        cla ; hold on ; axis equal ;

        % get agent, world, and planner
        A = S.agent ;
        W = S.worlds{world_index} ;
        P = S.planners{planner_index} ;

        if ~any(isinf(S.worlds{world_index}.bounds))
            axis(W.bounds)
        end

        color = S.planner_colors(planner_index,:) ;

        for plot_idx = S.plot_order
            switch plot_idx
                case 'A'
                    A.plot(color)
                case 'W'
                    W.plot()
                case 'P'
                    P.plot(color)
                otherwise
                    error(['Simulator plot order is broken! Make sure ',...
                          'it is a string containing the characters ',...
                          'A (for agent), W (for world), and P (for ',...
                          'planner), in the order you want them to ',...
                          'plot (WAP is the default).'])
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
                if S.manually_resize_gif
                    S.vdisp(['Please resize the figure to the size you ',...
                             'want saved, or hit any key to continue.'])
                    pause
                end
            end

            frame = getframe(fh) ;
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);

            filename = [S.save_gif_filename,'_planner_',num2str(planner_index),'.gif'] ;

            if S.start_gif
                imwrite(imind,cm,filename,'gif', 'Loopcount',inf,...
                        'DelayTime',S.save_gif_delay_time) ;
                S.start_gif = false ;
            else
                imwrite(imind,cm,filename,'gif','WriteMode','append',...
                        'DelayTime',S.save_gif_delay_time) ;
            end
        end
    end
    
    function plot_at_time(S,t,planner_index,world_index)
        % method: plot_at_time(t)
        %
        % Plot the agent, world, and planner at the time t; this calls the
        % plot_at_time method for each of class.
        
        if nargin < 4
            world_index = 1 ;
        end
        
        if nargin < 3
            planner_index = 1 ;
        end
        
        if nargin < 2
            t = 0 ;
        end
        
        % get agent, world, and planner
        A = S.agent ;
        W = S.worlds{world_index} ;
        P = S.planners{planner_index} ;
        
        % set up hold
        if ~ishold
            hold on
            hold_check = true ;
        else
            hold_check = false ;
        end
        
        for plot_idx = S.plot_order
            switch plot_idx
                case 'A'
                    A.plot_at_time(t)
                case 'W'
                    W.plot_at_time(t)
                case 'P'
                    P.plot_at_time(t)
                otherwise
                    error(['Simulator plot order is broken! Make sure ',...
                        'it is a string containing the characters ',...
                        'A (for agent), W (for world), and P (for ',...
                        'planner), in the order you want them to ',...
                        'plot (WAP is the default).'])
            end
        end
        
        if hold_check
            hold off
        end
    end
    
    %% animate
    function animate(S,planner_index,world_index,save_animation_gif)
    % method: S.animate(planner_index,world_index,save_gif)
    %         S.animate(save_gif)
    %
    % Animate the agent, world, and planner for the duration given by
    % A.time. The time between animated frames is given by the simulator's
    % animation_time_discretization property. The planner and world to
    % animate are given by the planner_index and world_index inputs, so the
    % simulator plots S.worlds{world_index} and S.planners{planner_index}.
    %
    % One can also call this as S.animate(true) to save a GIF for planner 1
    % and world 1 (which is useful if there's only one planner and world
    % currently set up in the simulator).
    
        % parse input arguments
        if nargin == 2 && islogical(planner_index)
            save_animation_gif = planner_index ;
            planner_index = 1 ;
            world_index = 1 ;
        else
            if nargin < 4
                save_animation_gif = false ;
                start_animation_gif = false ;
            end

            if nargin < 3
                world_index = 1 ;
            end

            if nargin < 2
                planner_index = 1 ;
            end
        end
        
        if save_animation_gif
            start_animation_gif = true ;
            filename = S.animation_gif_setup() ;
        end
        
        % get agent
        A = S.agent ;
        
        % get time
        t_vec = A.time(1):S.animation_time_discretization:A.time(end) ;

        % set hold
        if ~ishold
            hold on
            hold_check = true ;
        else
            hold_check = false ;
        end
        
        for t_idx = t_vec
            % create plot
            S.plot_at_time(t_idx,planner_index,world_index)
            
            % create gif
            if save_animation_gif
                % get current figure
                fh = get(groot,'CurrentFigure') ;
                frame = getframe(fh) ;
                im = frame2im(frame);
                [imind,cm] = rgb2ind(im,256);
                
                if start_animation_gif
                    imwrite(imind,cm,filename,'gif', 'Loopcount',inf,...
                        'DelayTime',A.animation_time_discretization) ;
                    start_animation_gif = false ;
                else
                    imwrite(imind,cm,filename,'gif','WriteMode','append',...
                        'DelayTime',A.animation_time_discretization) ;
                end
            else
                pause(S.animation_time_discretization)
            end
        end
        
        if hold_check
            hold off
        end
    end
    
    function filename = animation_gif_setup(S)       
        filename = S.save_gif_filename ;

        dir_content = dir(pwd) ;
        filenames   = {dir_content.name} ;
        file_check  = any(cellfun(@(x) strcmp(filename,x),filenames)) ;
        filename_new = filename ;
        cur_int = 1 ;

        while file_check
            filename_new = [filename(1:end-4),'_',num2str(cur_int),filename(end-3:end)] ;
            file_check  = any(cellfun(@(x) strcmp(filename_new,x),filenames)) ;
            cur_int = cur_int + 1 ;
        end

        filename = filename_new ;
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
