classdef segwayFRSplanner < planner2D
    properties
    % implementation-specific properties
        wpf
        v_dn
        v_up
        v_avg
        v_avg_cnt
        vmax_hi
        vmax_lo
        bounds_as_obstacle
        user_specified_buffer
        
    % trajectory optimization parameters
        point_spacing
        current_FRS
        FRS_slow
        FRS_fast
% % %         wstr_slow
% % %         wstr_fast
        traj_opt_prob_params
        cost_location_weight
        cost_heading_weight
        cost_traj_end_slow
        cost_traj_end_fast
        kstop
        kbounds_fast
        kbounds_slow
        
    % plan info
        saved_kopt
        saved_current_FRS
        saved_v
        current_kopt
        spinflag    
        current_obstacles_local
        current_pose
        n_stopped % to be used to force the simulator to stop
    end
    
    methods
    %% constructor
    function P = segwayFRSplanner(timeout,buf,verbose_level,name)
        if nargin < 3
            verbose_level = 0 ;
        end
        
        if nargin < 2
            buf = NaN ;
        end
        
        if nargin < 4
            name = 'FRS' ;
        end
        
        P@planner2D(timeout,verbose_level) ;
        P.name = name ;
        
        % load slow and fast FRS files
        P.FRS_slow = load('segwayFRS_deg12_vmax0p5_D1p5_withUnc.mat') ;
        P.FRS_fast = load('segwayFRS_deg12_vmax1p5_D1p5_withUnc.mat') ;
        
        % FRS parameters
        P.user_specified_buffer = buf ;
        P.t_move = 0.5 ;
        P.v_dn = 0.1 ;
        P.v_up = 0.45 ;
        P.v_avg_cnt = 3 ;
        P.vmax_hi = 1.5 ;
        P.vmax_lo = 0.5 ;
        P.kstop = [0 ; -1] ;
        P.current_kopt = P.kstop ;
        P.n_stopped = 0 ;
    end
    
    %% setup
    function setup(P,A,W)
    % calculate required buffer
        robot_buffer = A.footprint ;
        FRS_buffer = P.FRS_slow.rcar ;
        user_buffer = P.user_specified_buffer ;
        
        % the inner buffer is used to determine the point spacing required
        % by the FRS traj opt algorithm; this is dependent upon the size of
        % the segway's footprint in the FRS solution, not the segway's real
        % size
        inner_buffer_maximum = min(user_buffer, FRS_buffer) ;
        inner_buffer = max(inner_buffer_maximum,0.001) ;
        a = acos((FRS_buffer-inner_buffer)/FRS_buffer) ;
        P.point_spacing = 1.5*sin(a).*FRS_buffer ;
        
        % the outer buffer is how much to actually buffer obstacles by, in
        % order to compensate for the segway's actual size, since its FRS
        % representation is smaller actual size; we subtract the FRS buffer
        % size because the FRS applies to the entire area of the segway's
        % footprint, as opposed to the center of mass (which is what most
        % methods care about)
        outer_buffer = inner_buffer + (robot_buffer - FRS_buffer) ;
        P.default_buffer = outer_buffer ;
        
        % create bounds of room as an obstacle
        P.bounds = W.bounds + outer_buffer.*[1 -1 1 -1] ;
        xlo = P.bounds(1) ; xhi = P.bounds(2) ;
        ylo = P.bounds(3) ; yhi = P.bounds(4) ;
        
        Oin = [xlo, xhi, xhi, xlo, xlo ;
               ylo, ylo, yhi, yhi, ylo] ;
        
        P.bounds_as_obstacle = increasePolylineDensity(Oin(:,end:-1:1),...
                                                       P.point_spacing);

    % waypoint finder setup
        strt = W.start(1:2) ;
        goal = W.goal ;
        P.wpf = waypointFinder2D(strt, goal, 'gridSpacing', 0.25,...
                                'gridtype','rect','costMax',10000,...
                                'costIncrease',100,'costDecrease',0,...
                                'penaltyBuffer',0.1,...
                                'pathDiscount',0,...
                                'gridConnectivity',9,...
                                'gridBounds',P.bounds) ;
                            
    % trajectory optimization setup
% % %         % get w(z,k) polynomials as structures
% % %         P.wstr_slow = decompose_w_polynomial(P.FRS_slow.out.w) ;
% % %         P.wstr_fast = decompose_w_polynomial(P.FRS_fast.out.w) ;
% % %         
% % %         % create symbolic trajectory function
% % %         P.vdisp('Computing symbolic trajectory',2)
        x0 = P.FRS_slow.x0 ;
        y0 = P.FRS_slow.y0 ;
        T = P.FRS_slow.T ;
        D = P.FRS_slow.D ;
% % %         
% % %         syms w0 v0 xdes ydes hdes wdes vdes s real
% % %         
% % %         dt = 0.01 ;
% % %         z = [x0;y0;0;w0;v0] ;
% % %         
% % %         for tidx = 0:dt:T
% % %         % calculate the derivative of the dynamics
% % %             k1 = symbolic_segway_dynamics(z, wdes,vdes,w0,v0) ;
% % %             k2 = symbolic_segway_dynamics(z + (dt/2).*k1, wdes,vdes,w0,v0) ;
% % %             k3 = symbolic_segway_dynamics(z + (dt/2).*k2, wdes,vdes,w0,v0) ;
% % %             k4 = symbolic_segway_dynamics(z + dt.*k3, wdes,vdes,w0,v0) ;
% % % 
% % %         % increment z using RK4
% % %             z = z + ((T*dt)/(6*D))*s*(k1 + 2*k2 + 2*k3 + k4) ;
% % %         end
% % %         
% % %         
% % %         % create symbolic cost function
% % %         P.vdisp('Creating traj opt cost as a function',3)
        P.cost_heading_weight = 0.1 ;
        P.cost_location_weight = 1 ;
% % %         syms xdes ydes hdes real
% % %         
% % %         lw = P.cost_location_weight ; hw = P.cost_heading_weight ;
% % %         xy = z(1:2) ; hh = z(3) ;
% % %         C = lw.*(xy - [xdes;ydes]).^2 + hw.*(1-cos(hh-hdes)) ;
% % %         dCdk = diff(C,
% % %         
% % %         Cfn = matlabFunction(C,'Vars',[w0,v0,xdes,ydes,hdes,wdes,vdes,s]) ;
        s = msspoly('s',1) ; % free final time variable
        fslow = P.FRS_slow.f ;
        ffast = P.FRS_fast.f ;
        
        x = P.FRS_slow.x ;
        y = P.FRS_slow.y ;
        k = P.FRS_slow.k ;
        
        
        z = [x;y] ;
        
        F0 = (1/D).*[x0;y0;0] ;
        FTslow = F0 ;
        FTfast = F0 ;
        n_sim = 300 ;
        dt = T / n_sim ;
        for idx = 1:n_sim
            FTslow = FTslow + (dt/D)*s*[subs(fslow,z,FTslow(1:2));k(1)] ;
            FTfast = FTfast + (dt/D)*s*[subs(ffast,z,FTfast(1:2));k(1)] ;
        end
        
        P.cost_traj_end_slow = FTslow ;
        P.cost_traj_end_fast = FTfast ;
        
        probParams.w = P.FRS_slow.out.w ;
        probParams.z = [x;y] ;
        probParams.k = k ;
        probParams.s = s ;
        probParams.T = T ;
        probParams.max_fmincon_func_evals = 100000 ; % usually set to 600
        probParams.max_fmincon_iterations = 100000 ;
        probParams.optimality_tolerance = 10e-3 ;
        probParams.tsolve = P.timeout ;
        
        % set up the solver bounds based on the allowed speeds
        vmax_fast = P.FRS_fast.vmax ;
        vmax_allowed = min(A.vmax, vmax_fast) ;
        k2_max_allowed = (2/vmax_fast)*(vmax_allowed - vmax_fast/2) ;
        
        P.kbounds_fast = [-1 1; -1 k2_max_allowed] ;
        P.kbounds_slow = [-1 1; -1 1] ;
        probParams.kbounds = P.kbounds_slow ;
        
        % finalize setup of probParams in planner object
        P.traj_opt_prob_params = probParams ;
        
        P.current_FRS = 'FRS_slow' ;
        P.v_avg = NaN ;
        P.saved_kopt = [] ;
        P.saved_current_FRS = [] ;
        P.saved_v = [] ;
        
        P.spinflag = true ;
    end
    
    function update(P,~,W)
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
                            
        % reset probParams to the slow FRS
        P.current_FRS = 'FRS_slow' ;
        P.traj_opt_prob_params.w = P.FRS_slow.out.w ;
        P.v_avg = NaN ;
        P.saved_kopt = [] ;
        P.saved_current_FRS = [] ;
        P.saved_v = [] ;
        P.spinflag = true ;
        P.n_stopped = 0 ;
    end
    
    %% replan
    function [Tout,Uout,Zout] = replan(P,A,O)
        % get info of current FRS
        FRS = P.current_FRS ;
        D = P.(FRS).D ;
        x0 = P.(FRS).x0 ;
        y0 = P.(FRS).y0 ;
        
        % get state information from agent
        z0 = A.state(:,end) ;
        xy = z0(A.xy_state_indices) ;
        h = z0(A.heading_state_index) ;
        P.current_pose = [xy ; h] ;
        
        % assuming the obstacles are boxes, buffer them and increase the
        % density of points on them
        if ~isempty(O)
            O = bufferBoxObstacles(O,P.default_buffer) ;
            O = increasePolylineDensity(O,P.point_spacing) ;
        end
        
        % get the desired current lookahead distance
        lookahead_distance = z0(end) + P.(FRS).vmax ;
        
        % create waypoints based on current obstacles and agent position
        P.wpf.z = z0(A.xy_state_indices) ;
        P.wpf.updateGraphWeights(O) ;
        P.wpf.getWaypoints(true) ;
        [wp,hdes] = P.wpf.makeWaypoint(lookahead_distance) ;
        P.current_waypoint = wp ;
        wplocal = P.globalToLocal(wp,z0(A.xy_state_indices),h) ;
        hlocal = hdes - h ;
        
        % add the room boundary to the obstacles (we do this after waypoint
        % finding, since the room boundary messes with the waypoint finder)
        O = [O, nan(2,1), P.bounds_as_obstacle] ;
        P.current_obstacles = O ;
        
        % get the heading to the current waypoint
        h2wp = atan2(wplocal(2), wplocal(1) - x0) ;
        
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
            % rotate and shift the obstacles into the local reference frame,
            % then filter them by the unit circle and by any points inside
            % the footprint of the segway (if we have these points in
            % simulation, something broke horrible)
            Olocal = P.globalToLocal(O,z0(A.xy_state_indices),h) ;
            Odists = distPointToPoints((1/D)*[x0;y0], Olocal) ;
            Olog = Odists > (P.(FRS).rcar/D) & Odists < 1 ;

            % remove obstacle points behind the robot
            Olog = Olog & (Olocal(1,:) >= x0/D) ;
            Olocal = Olocal(:,Olog) ;
            
            % move away points in Pcur_filtered that are too close to the left and
            % right sides of the robot, and are not very far in front of the robot
            % (i.e. these points are never going to be reachable unless the robot
            % translates left or right by magic or h*ckin' big disturbance)
            if strcmp(FRS,'FRS_slow')
                small_distance_in_front_of_robot = 0.1 ; % changed 9:28PM 6 Nov 2017
            else
                small_distance_in_front_of_robot = 0.05 ;
            end
            
            rcar = P.(FRS).rcar ;
            
            scaling_factor_to_move_LR_points = 0.27/(rcar/D+0.05) ;
            Olog = Olocal(1,:) <= (x0/D + rcar/D + small_distance_in_front_of_robot) ;
            Olocal(2,Olog) = scaling_factor_to_move_LR_points.*Olocal(2,Olog) ;            

            % finally, filter out everything outside the [-1,1] box just in
            % case (this should be unnecessary)
            Olocal = filterPoints(0,0,Olocal,1) ;
            P.current_obstacles_local = Olocal ;

            % create cost function
            lw = P.cost_location_weight ;
            hw = P.cost_heading_weight ;

            if strcmp(FRS,'FRS_slow')
                FT = P.cost_traj_end_slow ;
                kbounds = P.kbounds_slow ;
            else
                FT = P.cost_traj_end_fast ;
                kbounds = P.kbounds_fast ;
            end

            C = lw.*(sum((FT(1:2) - wplocal).^2)) + ...
                hw.*((FT(3) - hlocal).^2) ;

            % update probParams object
            P.traj_opt_prob_params.w = P.(FRS).out.w ; % in case it was swapped
            P.traj_opt_prob_params.C = C ;
            P.traj_opt_prob_params.P = Olocal ;
            P.traj_opt_prob_params.k0 = [sign(h2wp);0] ;
            P.traj_opt_prob_params.s0 = 1 ;
            P.traj_opt_prob_params.kbounds = kbounds ;

            % run solver
            P.traj_opt_prob_params.tsolve = P.timeout ;
            
            if P.n_stopped > 5
               error('Stopped too many times!') 
            end
            
            try
                [kopt_new,~,exitflag] = findParamsTimeScaling(P.traj_opt_prob_params) ;
%                 [kopt_new,~,exitflag] = findParamsTOMLAB(P.traj_opt_prob_params) ;
                if abs(kopt_new(2) + 1) < 0.075
                    P.n_stopped = P.n_stopped + 1 ;
                    P.spinflag = true ;
                end
            catch
                kopt_new = [P.kstop ; 0] ;
                exitflag = 0 ;
                P.n_stopped = P.n_stopped + 1 ;
                P.spinflag = true ;
            end

            % create control output
            if isempty(kopt_new) || exitflag < 1
                P.current_kopt(2) = -1 ;
            else
                P.current_kopt = kopt_new(1:2) ;
            end

            P.saved_kopt = [P.saved_kopt, P.current_kopt] ;
            P.saved_current_FRS = [P.saved_current_FRS, P.(FRS).vmax] ;

            wdes = P.current_kopt(1)*P.(FRS).hdmax ;
            vmax = P.(FRS).vmax ;
            vdes = (vmax/2).*P.current_kopt(2) + (vmax/2) ;
            P.saved_v = [P.saved_v, vdes] ;

            % create output time, reference trajectory, and input
            [Tout,Zout] = A.odesolver(@(t,z) dubins_dynamics(z,wdes,vdes),...
                                      [0 P.t_move], z0(1:3)) ;
            Tout = Tout' ;
            Zout = [Zout' ; zeros(1,length(Tout)); vdes.*ones(1,length(Tout))] ;
            Uout = repmat([wdes;vdes],1,length(Tout)) ;        
        end
        
        P.xy_plan = Zout ;
        
        % swap FRS if needed
        if size(P.saved_v,2) >= P.v_avg_cnt
            P.v_avg = mean(P.saved_v(end-(P.v_avg_cnt-1):end)) ;
            if strcmp(FRS,'FRS_slow') && P.v_avg >= P.v_up
                P.vdisp('Swapping to faster FRS.',1)
                P.current_FRS = 'FRS_fast' ;
            elseif strcmp(FRS,'FRS_fast') && P.v_avg <= P.v_dn
                P.vdisp('Swapping to slower FRS.',1)
                P.current_FRS = 'FRS_slow' ;
            end
        end
        
        % save info
        P.info = {P.saved_kopt, P.saved_current_FRS} ;
    end
    
    %% utility functions
    function q = globalToLocal(P,p,z,h)
        % Method: q = globalToLocal(p,z,h)
        %
        % Rotate and shift the global points p (2-by-n) to the local frame
        % of the segway, given the current position z and heading h
        
        % get FRS info from the planner given the current FRS
        FRS = P.current_FRS ;
        D = P.(FRS).D ;
        x0 = P.(FRS).x0 ;
        y0 = P.(FRS).y0 ;
        
        % make rotation matrix
        R = [cos(h), sin(h) ; -sin(h), cos(h)] ;
        
        % create output
        q = R*((1/D)*(p - repmat(z,1,size(p,2)))) + repmat((1/D)*[x0;y0],1,size(p,2)) ;
    end
    
    function p = localToGlobal(P,q,z,h)
        % Method: q = localToGlobal(q,z,h)
        %
        % Rotate and shift the local points q (2-by-n) to the global frame
        % of the segway, given the current position z and heading h
    
        % get FRS info from the planner given the current FRS
        FRS = P.current_FRS ;
        D = P.(FRS).D ;
        x0 = P.(FRS).x0 ;
        y0 = P.(FRS).y0 ;
        
        % make rotation matrix
        R = [cos(h), sin(h) ; -sin(h), cos(h)] ;
        
        % create output
        N = size(q,2) ;
        p = D.*R'*(q - repmat((1/D).*[x0;y0],1,N)) + repmat(z,1,N) ;
    end
    
    function plotInLoop(P,n,c)
        if nargin < 3
            c = [0 0 1] ;
        end
        
        plotInLoop@planner2D(P,n,c)
        
        figure(n)
        
        % setup for plotting
        xcirc = cos(linspace(0,2*pi)) ;
        ycirc = sin(linspace(0,2*pi)) ;
        r = P.FRS_slow.rcar ;
        xvals = r.*xcirc ; yvals = r.*ycirc ;
        xy = P.current_pose(1:2) ; h = P.current_pose(3) ;
        
        % plot obstacle points
        Olocal = P.current_obstacles_local ;
        if ~isempty(Olocal)
            O = P.localToGlobal(Olocal,xy,h) ;
            plot(O(1,:),O(2,:),'.','Color',[1 0.5 0.5])
        end
        
        % plot FRS footprint at end of planned trajectory
        if ~isempty(P.xy_plan)
            xy = P.xy_plan(1:2,end) ;
            plot(xvals + xy(1), yvals + xy(2), ':','Color',c)
        end
        
        % plot waypointFinder2D output
        w = P.wpf ;
        plot(w.w(1,:),w.w(2,:),'--','Color',[1 0.8 0.5])
        plot(P.current_waypoint(1),P.current_waypoint(2),'ro')
    end
    end
end

% function zd = symbolic_segway_dynamics(z,wdes,vdes,w0,v0)
%     % extract the states
%     h = z(3) ;
%     w = z(4) ;
%     v = z(5) ;
%     
%     % determine the inputs
%     Kg = 2.95 ;
%     Ka = 3.0 ;
%     
%     g = Kg*(wdes - w) ;
%     a = Ka*(vdes - v) ;
%     
%     % calculate the derivatives
%     xd = v*cos(h) ;
%     yd = v*sin(h) ;
%     hd = w ;
%     wd = g ;
%     vd = a ;
%     
%     % make the output
%     zd = taylor([xd ; yd ; hd ; wd ; vd], [v0;w0;vdes;wdes], 'Order', 7) ;
% end

function zd = dubins_dynamics(z,wdes,vdes)
    zd = [vdes*cos(z(3)) ;
          vdes*sin(z(3)) ;
          wdes] ;
end
