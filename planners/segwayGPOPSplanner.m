classdef segwayGPOPSplanner < planner2D
    properties
    % implementation-specific properties
        gpops_problem
        t_plan_min % minimum amount of time to plan for
        t_plan_max % maximum "
        C % parallel cluster so we can run jobs with a timeout
        wpf
        spinflag % spin to face the first waypoint in the first iteration
        T_old
        U_old
        Z_old % plan from previous iteration
        use_coarse_initial_guess
    end
    
    methods
    %% constructor
    function P = segwayGPOPSplanner(t_min,t_max,timeout,verbose_level,...
                                    use_coarse_initial_guess,name)
        if nargin < 1
            t_min = 0.5 ;
        end
        
        if nargin < 2
            t_max = 2 ;
        end
        
        if nargin < 3
            timeout = 300 ;
        end
        
        if nargin < 4
            verbose_level = 0 ;
        end
        
        if nargin < 5
            use_coarse_initial_guess = true ;
        end
        
        if nargin < 6
            name = 'GPOPS' ;
        end
        
        P@planner2D(timeout,verbose_level) ;
        P.name = name ;
        P.t_move = t_min ;
        P.t_plan_min = t_min ;
        P.t_plan_max = t_max ;
        P.timeout = timeout ;
        P.use_coarse_initial_guess = use_coarse_initial_guess ;
    end
    
    %% setup
    function setup(P,A,W)
        % determine world buffer and bounds
        P.default_buffer = A.footprint + 0.02 ;
        b = P.default_buffer ;
        P.bounds = W.bounds + [b -b b -b] ;
        
        % make waypoint finder
        strt = W.start(1:2) ;
        goal = W.goal ;
        P.wpf = waypointFinder2D(strt, goal, 'gridSpacing', 0.25,...
                                'gridtype','rect','costMax',10000,...
                                'costIncrease',100,'costDecrease',0,...
                                'penaltyBuffer',0.25,...
                                'pathDiscount',0,...
                                'gridConnectivity',9,...
                                'gridBounds',P.bounds) ;
                            
        % make GPOPS problem object
        P.gpops_problem = P.make_GPOPS_problem_object(A,W) ;
        P.spinflag = true ; % reset the spinflag

        % call IPOPT to make sure GPOPS doesn't crash
        try
            ipopt
        catch
        end
    end
    
    function update(P,A,W)
        % update the waypoint finder
        strt = W.start(1:2) ;
        goal = W.goal ;
        P.wpf = waypointFinder2D(strt, goal, 'gridSpacing', 0.25,...
                                'gridtype','rect','costMax',10000,...
                                'costIncrease',100,'costDecrease',0,...
                                'penaltyBuffer',0.25,...
                                'pathDiscount',0,...
                                'gridConnectivity',9,...
                                'gridBounds',P.bounds) ;
        
        % setup for GPOPS
        P.gpops_problem = P.make_GPOPS_problem_object(A,W) ;
        P.spinflag = true ; % reset the spinflag
        P.T_old = [] ; P.U_old = [] ; P.Z_old = [];
        try
            ipopt
        catch
        end
 
    end
    
    %% replan
    function [Tout,Uout,Zout] = replan(P,A,O,~)
        % put obstacles into problem
        O = bufferBoxObstacles(O,P.default_buffer) ;
        P.current_obstacles = O ;
        P.gpops_problem.auxdata.obs = O ;
        
        % get state information from agent
        z0 = A.state(:,end) ;
        xy = z0(A.xy_state_indices) ;
        
        % generate new waypoint
        lookahead_distance = z0(end) + A.vmax ;
        P.wpf.z = xy ;
        P.wpf.updateGraphWeights(O) ;
        P.wpf.getWaypoints(true) ;
        [wp,hdes] = P.wpf.makeWaypoint(lookahead_distance) ;
        P.current_waypoint = wp ;
        
        P.gpops_problem.auxdata.goal = wp ;
        P.gpops_problem.auxdata.hdes = hdes ;
        
        % set up planner problem
        P.gpops_problem.bounds.phase.initialstate.lower = z0' ;
        P.gpops_problem.bounds.phase.initialstate.upper = z0' ;
        
        % reset timer
        P.gpops_problem.auxdata.timer_start = tic ;
        
        % initial guess
        if (isempty(P.Z_old) || P.spinflag) || P.use_coarse_initial_guess
            x_guess = [z0(1); z0(1) + z0(5)] ;
            y_guess = [z0(2); z0(2)] ;
            h_guess = [z0(3); z0(3)] ;
            hd_guess = [0;0] ;
            v_guess = [z0(5); z0(5)] ;

            P.gpops_problem.guess.phase.state = [x_guess, y_guess, h_guess, hd_guess,  v_guess];
        else
            % find closest point in Zold to current state
            T = P.T_old ; U = P.U_old ; Z = P.Z_old ;
            xy_old = Z(1:2,:) ;
            [~,idx] = min(distPointToPoints(xy,xy_old)) ;
            
            % get the xy difference and shift Z by that much
            dxy = xy - xy_old(:,1) ;
            Z(1:2,:) = Z(1:2,:) + repmat(dxy,1,size(Z,2)) ;
            
            T = T(idx:end) - T(idx) ;
            U = U(:,idx:end) ;
            Z = Z(:,idx:end) ;
            
            P.gpops_problem.guess.time = T' ;
            P.gpops_problem.guess.control = U' ;
            P.gpops_problem.guess.state = Z' ;
        end

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
            gp = P.gpops_problem ;
            planning_time = tic ;
            output = gpops2(gp) ;
            planning_time = toc(planning_time) ;
            if planning_time <= P.timeout
                solution = output.result.solution ;
                Tout = solution.phase.time' ;
                Uout = solution.phase.control' ;
                Zout = solution.phase.state' ;
            else
                Tout = [] ; Uout = [] ; Zout = [] ;
            end
        end

        if ~isempty(Zout)
            P.xy_plan = Zout(A.xy_state_indices,:) ;
        end
    end
    
    %% make GPOPS problem object
    function out = make_GPOPS_problem_object(P,A,W)
        bounds.phase.initialtime.lower = 0 ;
        bounds.phase.initialtime.upper = 0 ;
        bounds.phase.finaltime.lower   = P.t_plan_min ;
        bounds.phase.finaltime.upper   = P.t_plan_max ;

        % problem bounds
        b = P.default_buffer ;
        B = W.bounds + [b -b b -b] ;
        xmin = B(1) ;
        xmax = B(2) ;
        ymin = B(3) ;
        ymax = B(4) ;
        hmin = -Inf ;
        hmax = +Inf ;
        wmin = -A.wmax ;
        wmax = +A.wmax ;
        vmin = 0 ;
        vmax = A.vmax ;

        bounds.phase.state.lower        = [ xmin, ymin, hmin, wmin, vmin ];
        bounds.phase.state.upper        = [ xmax, ymax, hmax, wmax, vmax ];
        bounds.phase.finalstate.lower   = -Inf * ones(1, 5);
        bounds.phase.finalstate.upper   = Inf * ones(1, 5);
        bounds.phase.control.lower = [ wmin, vmin];
        bounds.phase.control.upper = [ wmax, vmax];
        bounds.phase.path.lower = 0 ;
        bounds.phase.path.upper = Inf ;

        % default initial guess
        t_guess    = [0; 1 ];
        w_guess    = [0; 0 ];
        v_guess    = [0; 0 ];
        guess.phase.time = t_guess ;
        guess.phase.control = [w_guess,v_guess]; 

        % mesh settings
        mesh.method = 'hp-LiuRao-Legendre';
        mesh.tolerance = 1e-6 ; 
        mesh.maxiterations = 1000 ;
        mesh.colpointsmin = 4 ;
        mesh.colpointsmax = 10 ;
        mesh.phase.colpoints = 4*ones(1,10);
        mesh.phase.fraction = 0.1*ones(1,10);

        % obstacle gpops_problem
        auxdata.location_weight = 1 ;
        auxdata.heading_weight = 0.1 ;
        auxdata.timer_start = tic ;
        auxdata.timeout = P.timeout ;

        % finalize setting up GPOPS problem object
        out.bounds = bounds ;
        out.guess = guess ;
        out.mesh = mesh ;
        out.name = 'segway_simulation' ;
        out.functions.continuous = @dynamics_function ;
        out.functions.endpoint = @endpoint_function ;
        out.auxdata = auxdata ;
        out.nlp.solver = 'ipopt' ;
        out.displaylevel = 0 ;
        out.derivatives.derivativelevel = 'second' ;
        out.derivatives.supplier = 'sparseCD' ;
    end
    
    %% plotting
    function plotInLoop(P,n,c)
        if nargin < 3
            c = [0 0 1] ;
        end
        
        plotInLoop@planner2D(P,n,c)
        
        figure(n)
        
        % plot obstacle points
        O = P.current_obstacles ;
        if ~isempty(O)
            plot(O(1,:),O(2,:),'-','Color',[1 0.7 0.7])
        end
        
        % waypoint finder
        w = P.wpf ;
        plot(w.w(1,:),w.w(2,:),'--')
        plot(P.current_waypoint(1),P.current_waypoint(2),'ro')
    end
    end
end

function out = dynamics_function(input)
% dynamics
    x = input.phase.state(:, 1);
    y = input.phase.state(:, 2);
    h = input.phase.state(:, 3);
    hd = input.phase.state(:, 4);
    v = input.phase.state(:, 5);

    k1 = input.phase.control(:, 1);
    k2 = input.phase.control(:, 2);

    % yaw rate input
    wdes = k1 ;
    Kg = 2.95 ;
    g = Kg*(wdes - hd) ;

    % acceleration input
    vdes = k2 ;
    vdelta = vdes - v ;
    Ka = 3 ;
    a = Ka*vdelta ;

    xdot     = v .* cos(h) ;
    ydot     = v .* sin(h) ;
    thetadot = hd ;
    omegadot = g ;
    vdot = a ;

    out.dynamics  = [xdot, ydot, thetadot, omegadot, vdot];

% obstacle check
    O = input.auxdata.obs ;
    
    % make sure the last column is nans so the indexing works out correctly
    if ~isnan(O(1,end))
        O = [O, nan(2,1)] ;
    end

    % setup for halfplanes
    nx = length(x) ;

    % get obstacle start indices
    Nnan = isnan(O(1,:)) ;
    idxs = 1:size(O,2) ;
    idxs = idxs(Nnan) ;

    if idxs(1) ~= 1
    idxs = [1,idxs+1] ; % this gives us the first index of each obstacle
                        % and the last index in the obstacle variable (the
                        % indices are columns)
    end

    halfPlaneCheck = nan(nx,length(idxs)-1) ;

    % using a for loop to check each obstacle...
    for idx = 1:(length(idxs)-1)
        i1 = idxs(idx) ; % starting index of current obstacle
        i2 = idxs(idx+1) ; % starting index of next obstacle

        % get the start and end points corresponding to the current obstacle;
        % the obstacles are line segments that start from [Ax;Ay] and end at
        % [Bx;By]
        Ax = O(1,i1:(i2-3)) ; Ay = O(2,i1:(i2-3)) ;
        Bx = O(1,(i1+1):(i2-2)) ; By = O(2,(i1+1):(i2-2)) ;

        % the rows of this matrix correspond to each (x,y) point to be tested;
        % the columns correspond to each segment defining the current obstacle,
        % which is a counterclockwise polygon; each entry in this matrix thus
        % is negative if a point (x,y) is to the left or right a line segment
        halfPlaneCheckMatrix = x*(By-Ay) - y*(Bx-Ax) - ...
                               repmat(Ax.*By,nx,1) + repmat(Ay.*Bx,nx,1) ;

        halfPlaneCheck(:,idx) = max(halfPlaneCheckMatrix,[],2) ;
    end
    
    halfPlaneCheck = min(halfPlaneCheck,[],2) ;
    
    out.path = halfPlaneCheck ;
    
    if timeout_check(input)
        out.dynamics = zeros(nx,5) ;
        error('timed out!')
%         out.path = zeros(size(halfPlaneCheck)) ;
%     else
%         
    end
%     timeout_check(input) ;
end

function out = endpoint_function(input)
%     if timeout_check(input)
%         out.objective = 0 ;
%     else
        x = input.phase.finalstate(1);
        y = input.phase.finalstate(2);
        h = input.phase.finalstate(3);

        gx = input.auxdata.goal(1) ;
        gy = input.auxdata.goal(2) ;

        hdes = input.auxdata.hdes ;
        location_weight = input.auxdata.location_weight ;
        heading_weight = input.auxdata.heading_weight ;

        out.objective = location_weight.*((x-gx).^2 + (y-gy).^2) + ...
                           heading_weight.*((h - hdes).^2) ;
%     end
end

function out = timeout_check(input)
    out = toc(input.auxdata.timer_start) >= input.auxdata.timeout ;
end

% function timeout_check(input)
%     if toc(input.auxdata.timer_start) >= input.auxdata.timeout
%         error('Timed out!')
%     end
% end
