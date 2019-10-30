classdef RRT_star_HLP < high_level_planner
% Class: RRT_star_HLP < high_level_planner
%
% This implements RRT* as a high-level planner for the simulator framework.
% For now, it only works for 2D worlds/obstacles.
%
% Authors: Bohao Zhang and Shreyas Kousik
% Created: 5 July 2019
% Updated: 30 Oct 2019

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
        grow_new_tree_every_iteration_flag = false ;
        
        % tree data structures
        nodes
        nodes_parent
        cost
        
        % plot flags
        plot_tree_flag = false ;
        plot_best_path_flag = true ;
        plot_waypoint_flag = false ;
    end
    methods
        %% constructor
        function HLP = RRT_star_HLP(varargin)
            HLP@high_level_planner(varargin{:}) ;
            
            % set up plot data
            HLP.plot_data.tree = [] ;
            HLP.plot_data.best_path = [] ;
            HLP.plot_data.waypoint = [] ;
        end
        
        %% plan a path
        function grow_tree(HLP,agent_info,obstacles)
            % grow tree
            start_tic = tic ;
            t_cur = toc(start_tic) ;
            
            % get agent info
            z = agent_info.position(:,end) ;
            R = agent_info.sensor_radius ;
            
            if HLP.grow_new_tree_every_iteration_flag || isempty(HLP.nodes)
                HLP.nodes = z ;
                HLP.nodes_parent = 0 ;
                HLP.cost = 0 ;
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
                
                % if the new node is feasible...
                if HLP.node_feasibility_check(new_node,nearest_node,obstacles)
                    % get all nearby nodes within a fixed radius
                    near_distance_threshold = HLP.near_distance_max * ...
                        ((1+log(N_nodes))/N_nodes)^(1/HLP.dimension) ;
                    near_distances = vecnorm(HLP.nodes - repmat(new_node,1,N_nodes)) ;
                    near_dist_log = near_distances <= near_distance_threshold & ...
                                    near_distances > 0 ;
                    near_nodes = HLP.nodes(:,near_dist_log) ;
                    near_node_indices = node_indices(near_dist_log) ;
                    
                    
                    % check the cost of all the neighbors
                    near_cost = HLP.cost(near_dist_log) ;
                    [min_cost,min_cost_index] = min(near_cost) ;
                    cheapest_node = near_nodes(:,min_cost_index) ;
                    cheapest_node_index = near_node_indices(min_cost_index) ;
                    
                    % check the feasibility to all neighbors
                    near_feasibility = HLP.node_feasibility_check(new_node,near_nodes,obstacles) ;
                    
                    % check if connecting to the cheapest neighbor is
                    % feasible
                    N_nodes = N_nodes + 1 ;
                    node_indices = [node_indices, N_nodes] ;
                    HLP.nodes = [HLP.nodes, new_node] ;
                    if near_feasibility(min_cost_index)
                        % connect to the cheapest neighbor
                        HLP.nodes_parent = [HLP.nodes_parent,cheapest_node_index] ;
                        new_cost = min_cost + vecnorm(new_node - cheapest_node) ;
                    else
                        % connect to the previous nearest node
                        HLP.nodes_parent = [HLP.nodes_parent,nearest_node_index] ;
                        new_cost = HLP.cost(nearest_node_index) + NNGD ;
                    end
                    HLP.cost = [HLP.cost, new_cost] ;
                    
                    % rewire tree by examining all nearby nodes to see if
                    % connecting them to the new node reduces their cost
                    near_costs_old = HLP.cost(near_dist_log) ;
                    near_costs_new = new_cost + near_distances(near_dist_log) ;
                    near_costs_log = near_costs_new < near_costs_old ;
                    rewire_indices = near_node_indices(near_costs_log) ;
                    HLP.nodes_parent(rewire_indices) = N_nodes ;
                    HLP.cost(rewire_indices) = near_costs_new(near_costs_log) ;
                    
                    % % FOR DEBUGGING:
                    % rewired_nodes = near_nodes(:,near_costs_log) ;
                    % plot_path([nearest_node,new_node],'b-')
                    % plot_path(near_nodes,'r.')
                    % plot_path(rewired_nodes,'g.')
                    % pause(0.001)
                end
                
                t_cur = toc(start_tic) ;
            end
            
            % find the node closest to the goal
            distances_to_goal = vecnorm(HLP.nodes - repmat(HLP.goal,1,N_nodes)) ;
            [~,best_node_index] = min(distances_to_goal) ;
            
            % traverse from the closest node to the tree root
            HLP.best_path_indices = best_node_index ;
            current_index = best_node_index ;
            
            while current_index > 0
                current_parent_index = HLP.nodes_parent(current_index) ;
                HLP.best_path_indices = [HLP.best_path_indices, current_parent_index] ;
                current_index = current_parent_index ;
            end
            
            % switch path to go from start to goal
            HLP.best_path_indices = HLP.best_path_indices(end-1:-1:1) ;
            HLP.best_path = HLP.nodes(:,HLP.best_path_indices) ;
            HLP.best_path_distance = dist_polyline_cumulative(HLP.best_path) ;
        end
        
        %% get waypoint
        function wp = get_waypoint(HLP, agent_info, world_info, lookahead_distance)
            % get the agent pose
            z = agent_info.position(:,end) ;
            
            % call the RRT star algorithm
            HLP.grow_tree(agent_info, world_info.obstacles) ;
            
            % if the best path is non-empty...
            if ~isempty(HLP.best_path)
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
                    HLP.plot_data.nodes = plot_path(N,'b-') ;
                end
            end
            
            if HLP.plot_best_path_flag
                BP = HLP.get_points_for_plot(HLP.best_path) ;
                
                if check_if_plot_is_available(HLP,'best_path')
                    HLP.plot_data.best_path.XData = BP(1,:) ;
                    HLP.plot_data.best_path.YData = BP(2,:) ;
                else
                    HLP.plot_data.best_path = plot_path(BP,'g-','LineWidth',1.5) ;
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