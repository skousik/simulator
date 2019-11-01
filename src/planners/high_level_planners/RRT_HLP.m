classdef RRT_HLP < high_level_planner
% Class: RRT_HLP < high_level_planner
%
% This implements RRT as a high-level planner for the simulator framework.
% For now, it only works for 2D worlds/obstacles.
%
% Authors: Shreyas Kousik and Bohao Zhang
% Created: 31 July 2019
% Updated: 31 Oct 2019

    properties
        % timing and tree growth limits
        timeout = 0.25 ; % seconds
        final_goal_rate = 0.08;
        new_node_growth_distance = 0.5 ; % m
        near_distance_max = 2 ;
        best_path = [];
        best_path_indices = 0;
        best_path_distance
        bounds
        N_nodes_max = 40000 ;
        N_near_goal = 50 ;
        
        % method for growing the RRT:
        % 'new' means grow a new tree every planning iteration
        % 'seed' means use the previous best path to seed a new tree
        % 'keep' means keep growing the same tree every iteration
        grow_tree_mode = 'new' ; % pick 'new' or 'seed' or 'keep'
        
        % tree data structures
        nodes
        nodes_parent
        
        % plot flags
        plot_tree_flag = false ;
        plot_best_path_flag = true ;
        plot_waypoint_flag = false ;
    end
    methods
        %% constructor
        function HLP = RRT_HLP(varargin)
            HLP@high_level_planner(varargin{:}) ;
            
            % set up plot data
            HLP.plot_data.tree = [] ;
            HLP.plot_data.best_path = [] ;
            HLP.plot_data.waypoint = [] ;
        end
        
        %% plan a path
        function exit_flag = grow_tree(HLP,agent_info,obstacles)
            % grow tree
            start_tic = tic ;
            t_cur = toc(start_tic) ;
            
            % get agent info
            z = agent_info.position(:,end) ;
            R = agent_info.sensor_radius ;
            
            switch HLP.grow_tree_mode
                case 'new'
                    HLP.vdisp('Growing new tree!',8)
                    HLP.nodes = z ;
                    HLP.nodes_parent = 0 ;
                case 'seed'
                    HLP.vdisp('Seeding from best path!',8)
                    HLP.nodes = HLP.best_path ;
                    HLP.nodes_parent = 0:(length(HLP.nodes)-1) ;
                case 'keep'
                    HLP.vdisp('Keeping previous tree!',8)
                otherwise
                    error('Please pick a valid tree growth mode!')
            end
            
            if isempty(HLP.nodes)
                HLP.nodes = z ;
                HLP.nodes_parent = 0 ;
            end
            
            B = HLP.bounds ;
            NNGD = HLP.new_node_growth_distance ;
            N_nodes = size(HLP.nodes,2) ;
            node_indices = 1:N_nodes ;
            
            while t_cur < HLP.timeout && (length(HLP.nodes) < HLP.N_nodes_max)
                % generate a random node to explore by picking the
                % agent's location plus a random distance in a random
                % direction, which keeps the tree growing within the sensor
                % radius of the agent
                
                if rand < HLP.final_goal_rate
                    % choose the final goal as the new node direction with
                    % some probability
                    rand_dir = HLP.goal - z ;
                    rand_dist = R ;
                else
                    rand_dir = rand_range(-1,1,[],[],2,1) ;
                    rand_dist = rand_range(0,R) ;
                end
                rand_node = z + rand_dist.*make_unit_length(rand_dir) ;
                rand_node = bound_array_elementwise(rand_node(:)',B(1:2:end),B(2:2:end))' ;

                % find the nearest node in the tree
                node_distances = vecnorm(HLP.nodes - repmat(rand_node,1,N_nodes)) ;
                [nearest_node_distance,nearest_node_index] = min(node_distances) ;
                nearest_node = HLP.nodes(:,nearest_node_index) ;
                
                % grow the tree towards the direction new node
                if nearest_node_distance <= NNGD
                    new_node = rand_node;
                else
                    new_node_direction = rand_node - nearest_node ;
                    new_node_direction = new_node_direction / norm(new_node_direction) ;
                    new_node = nearest_node + NNGD.*new_node_direction ;
                end
                
                % if the new node is feasible, add it to the tree
                if HLP.node_feasibility_check(new_node,nearest_node,obstacles)
                    N_nodes = N_nodes + 1 ;
                    HLP.nodes = [HLP.nodes, new_node] ;
                    HLP.nodes_parent = [HLP.nodes_parent, nearest_node_index] ;
                end
                
                t_cur = toc(start_tic) ;
            end
            
            % find the node closest to the goal
            distances_to_goal = vecnorm(HLP.nodes - repmat(HLP.goal,1,N_nodes)) ;
            [~,best_node_index] = min(distances_to_goal) ;
            
            % get best path
            if best_node_index > 1
                HLP.best_path_indices = HLP.tree_path_root_to_node(best_node_index) ;
            else
                HLP.best_path_indices = 1 ;
            end
            
            % get best path and distance along it
            HLP.best_path = HLP.nodes(:,HLP.best_path_indices) ;
            HLP.best_path_distance = dist_polyline_cumulative(HLP.best_path) ;
            
            % set waypoints field in case a planner uses it
            HLP.waypoints = HLP.best_path ;
            
            % set up exit flag
            if HLP.best_path_distance(end) > 0
                exit_flag = 1 ;
            else
                exit_flag = 0 ;
            end
        end
        
        function path_indices = tree_path_root_to_node(HLP,destination_index)
            % initialize
            path_indices = destination_index ;
            current_index = destination_index ;
            
            while current_index > 0
                current_parent_index = HLP.nodes_parent(current_index) ;
                path_indices = [path_indices, current_parent_index] ;
                current_index = current_parent_index ;
            end
            
            % flip direction
            path_indices = path_indices(end-1:-1:1) ;
        end
        
        function path_indices = tree_path_node_to_node(HLP,source_index,destination_index)
            warning('The tree_path_node_to_node method doesn''t work right yet!')
            
            % get paths from root to source and destination
            path_root_to_source = HLP.tree_path_root_to_node(source_index) ;
            path_root_to_destination = HLP.tree_path_root_to_node(destination_index) ;
            
            % find the last node along both paths
            N_R2S = length(path_root_to_source) ;
            N_R2D = length(path_root_to_destination) ;
            comparison_array = repmat(path_root_to_source(:),1,N_R2D) == ...
                repmat(path_root_to_destination,N_R2S,1) ;
            join_index_R2D = find(any(comparison_array,1),1,'last') ;
            join_index_R2S = find(any(comparison_array,2),1,'last') ;
            
            path_indices = unique([path_root_to_source(end:join_index_R2S), ...
                path_root_to_destination(join_index_R2D:end)],'stable') ;
        end
        
        %% get waypoint
        function wp = get_waypoint(HLP, agent_info, world_info, lookahead_distance)
            % get the agent pose
            z = agent_info.position(:,end) ;
            
            % call the RRT star algorithm
            exit_flag = HLP.grow_tree(agent_info, world_info.obstacles) ;
            
            % if the best path is non-empty...
            if exit_flag
                % get the agent's distance along the best path
                d = dist_point_on_polyline(z,HLP.best_path) ;
                
                % interpolate to get the waypoint
                d_best = HLP.best_path_distance ;
                d_lkhd = min(d_best(end), d + lookahead_distance) ;
                wp = match_trajectories(d_lkhd,d_best,HLP.best_path) ;
            else
                wp = z ;
            end
            
            HLP.current_waypoint = wp;
        end
        
        %% node feasibility check
        function out = node_feasibility_check(HLP,node_source,node_target,obstacles)
            % this method should return TRUE if the node is feasible
            
            O = obstacles ;
            
            N = size(node_target,2) ;
            
            out = true(1,N) ; % optimism!
            
            % TO DO: vectorize this!
            if ~isempty(O)
                for index = 1:N
                    X = [node_source, node_target(:,index)] ;
                    [check_inside,~] = inpolygon(X(1,:)',X(2,:)',O(1,:)',O(2,:)') ;
                    [check_through,~] = polyxpoly(X(1,:)',X(2,:)',O(1,:)',O(2,:)') ;
                    out(index) = isempty(check_through) & ~any(check_inside) ;
                end
            end
        end
        
        %% plotting
        function plot(HLP)
            if HLP.plot_tree_flag
                N = HLP.get_points_for_plot(HLP.make_polyline_tree()) ;
                
                if isempty(N)
                    N = nan(2,1) ;
                end
                
                if check_if_plot_is_available(HLP,'tree')
                    HLP.plot_data.nodes.XData = N(1,:) ;
                    HLP.plot_data.nodes.YData = N(2,:) ;
                else
                    HLP.plot_data.nodes = plot_path(N,'-','Color',[0.5 0.5 1]) ;
                end
            end
            
            if HLP.plot_best_path_flag
                BP = HLP.get_points_for_plot(HLP.best_path) ;
                
                if check_if_plot_is_available(HLP,'best_path')
                    HLP.plot_data.best_path.XData = BP(1,:) ;
                    HLP.plot_data.best_path.YData = BP(2,:) ;
                else
                    HLP.plot_data.best_path = plot_path(BP,'--',...
                        'Color',[0.9 0.7 0.1],'LineWidth',1.5) ;
                end
            end
            
            if HLP.plot_waypoint_flag
                wp = HLP.get_points_for_plot(HLP.current_waypoint) ;
                
                if check_if_plot_is_available(HLP,'waypoint')
                    HLP.plot_data.waypoint.XData = wp(1,:) ;
                    HLP.plot_data.waypoint.YData = wp(2,:) ;
                else
                    HLP.plot_data.waypoint = plot_path(wp,'k*','LineWidth',1.5) ;
                end
            end
        end
        
        function P = make_polyline_tree(HLP)
            N = HLP.nodes ;
            NP = HLP.nodes_parent ;
            
            P = [N(:,NP(2:end)) ; N(:,2:end) ; nan(2,size(N,2)-1)] ;
            P = reshape(P,2,[]) ;
        end
        
        function p = get_points_for_plot(HLP,p_in)
            p = p_in ;
            if isempty(p)
                p = nan(2,1) ;
            end
        end
    end
end