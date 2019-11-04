classdef RRT_HLP < high_level_planner
    % Class: RRT_HLP < high_level_planner
    %
    % This implements RRT as a high-level planner for the simulator framework.
    % For now, it only works for 2D worlds/obstacles.
    %
    % Authors: Shreyas Kousik and Bohao Zhang
    % Created: 31 July 2019
    % Updated: 3 Nov 2019
    
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
        N_nodes ;
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
        
        function setup(HLP,agent_info,world_info)
            HLP.initialize_tree(agent_info) ;
            HLP.dimension = world_info.dimension ;
            HLP.goal = world_info.goal ;
            HLP.bounds = world_info.bounds ;
        end
        
        %% tree growth
        function exit_flag = grow_tree(HLP,agent_info,obstacles)
            % grow tree
            start_tic = tic ;
            t_cur = toc(start_tic) ;
            
            % get agent info
            z = agent_info.position(:,end) ;
            
            % initialize tree
            HLP.initialize_tree(agent_info) ;
            
            % grow tree until timeout or node limit is reached
            while t_cur < HLP.timeout && (HLP.N_nodes < HLP.N_nodes_max)
                [new_node,nearest_node,nearest_node_index] = HLP.make_new_node(agent_info) ;
                
                % make sure the new node and nearest node are not the same
                % (this can happen when the RRT is growing near the
                % boundaries of the workspace)
                if vecnorm(new_node - nearest_node) == 0
                    % dbstop in RRT_HLP at 94
                    continue
                end
                
                % if the new node is feasible, add it to the tree
                if HLP.node_feasibility_check(new_node,nearest_node,obstacles)
                    HLP.nodes = [HLP.nodes, new_node] ;
                    HLP.nodes_parent = [HLP.nodes_parent, nearest_node_index] ;
                    HLP.N_nodes = HLP.N_nodes + 1 ;
                end
                
                t_cur = toc(start_tic) ;
            end
            
            % after growing the tree, get the path that ends closest to the
            % goal
            exit_flag = HLP.find_best_path() ;
        end
        
        function initialize_tree(HLP,agent_info)
            z = agent_info.position(:,end) ;
            switch HLP.grow_tree_mode
                case 'new'
                    HLP.vdisp('Growing new tree!',8)
                    HLP.nodes = z ;
                    HLP.nodes_parent = 0 ;
                case 'seed'
                    HLP.vdisp('Seeding from previous best path!',8)
                    HLP.nodes = HLP.best_path ;
                    HLP.nodes_parent = 0:(length(HLP.nodes) - 1) ;
                case 'keep'
                    HLP.vdisp('Keeping previous tree!',8)
                otherwise
                    error('Please pick a valid tree growth mode!')
            end
            
            if isempty(HLP.nodes)
                HLP.nodes = z ;
                HLP.nodes_parent = 0 ;
            end
            
            HLP.N_nodes = size(HLP.nodes,2) ;
        end
        
        function [new_node,nearest_node,nearest_node_index] = make_new_node(HLP,agent_info)
            % get useful info
            NNGD = HLP.new_node_growth_distance ;
            
            % generate a random node
            rand_node = HLP.make_random_node(agent_info) ;
            
            % get the nearest node
            [nearest_node,nearest_node_distance,nearest_node_index] = HLP.find_nearest_node(rand_node) ;
            
            % grow the tree towards the direction new node
            if nearest_node_distance <= NNGD
                new_node = rand_node;
            else
                new_node_direction = rand_node - nearest_node ;
                new_node_direction = new_node_direction / norm(new_node_direction) ;
                new_node = nearest_node + NNGD.*new_node_direction ;
            end
        end
        
        function rand_node = make_random_node(HLP,agent_info)
            % generate a random node to explore by picking the
            % agent's location plus a random distance in a random
            % direction, which keeps the tree growing within the sensor
            % radius of the agent
            
            z = agent_info.position(:,end) ;
            R = agent_info.sensor_radius ;
            B = HLP.bounds ;
            
            if rand < HLP.final_goal_rate
                % choose the final goal as the new node direction with
                % some probability
                rand_dir = HLP.goal - z ;
                rand_dist = R ;
            else
                rand_dir = rand_range(-1,1,[],[],HLP.dimension,1) ;
                rand_dist = rand_range(0,R) ;
            end
            rand_node = z + rand_dist.*make_unit_length(rand_dir) ;
            rand_node = bound_array_elementwise(rand_node(:)',B(1:2:end),B(2:2:end))' ;
        end
        
        function [node,node_dist,node_idx] = find_nearest_node(HLP,node_in)
            % find the nearest node in the tree
            node_distances = vecnorm(HLP.nodes - repmat(node_in,1,HLP.N_nodes)) ;
            [node_dist,node_idx] = min(node_distances) ;
            node = HLP.nodes(:,node_idx) ;
        end
        
        %% path planning
        function exit_flag = find_best_path(HLP)
            % find the node closest to the goal
            distances_to_goal = vecnorm(HLP.nodes - repmat(HLP.goal,1,HLP.N_nodes)) ;
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
            hc = hold_switch() ;
            
            if HLP.plot_tree_flag
                T = HLP.get_points_for_plot(HLP.make_polyline_tree()) ;
                plot_object(HLP,T,'tree','-','Color',[0.5 0.5 1]) ;
            end
            
            if HLP.plot_best_path_flag
                BP = HLP.get_points_for_plot(HLP.best_path) ;
                plot_object(HLP,BP,'best_path','--',...
                        'Color',[0.7 0.5 0.2],'LineWidth',1.5) ;
            end
            
            if HLP.plot_waypoint_flag
                wp = HLP.get_points_for_plot(HLP.current_waypoint) ;
                plot_object(HLP,wp,'waypoint','k*','LineWidth',1.5) ;
            end
            
            hold_switch(hc) ;
        end
        
        function P = make_polyline_tree(HLP)
            N = HLP.nodes ;
            NP = HLP.nodes_parent ;
            d = HLP.dimension ;
            
            P = [N(:,NP(2:end)) ; N(:,2:end) ; nan(d,size(N,2)-1)] ;
            P = reshape(P,d,[]) ;
        end
        
        function p = get_points_for_plot(HLP,p_in)
            p = p_in ;
            if isempty(p)
                p = nan(2,1) ;
            end
        end
    end
end