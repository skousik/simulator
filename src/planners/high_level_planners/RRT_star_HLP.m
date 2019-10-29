classdef RRT_star_HLP < high_level_planner
% Class: RRT_star_HLP < high_level_planner
%
% This implements RRT* as a high-level planner for the simulator framework.
% For now, it only works for 2D worlds/obstacles.
%
% Authors: Bohao Zhang and Shreyas Kousik
% Created: 5 July 2019
% Updated: 28 Oct 2019

    properties
        timeout = 0.25 ; % seconds
        final_goal_rate = 0.08;
        new_node_growth_distance = 0.1 ; % m
        near_distance_rate = 6;
        change_plan_distance = 0.6;
        plan = [];
        plan_index = 0;
        bounds
        
        grow_new_tree_every_iteration_flag = true ;
        
        current_nodes
        current_nodes_parent
        current_cost
    end
    methods
        %% constructor
        function HLP = RRT_star_HLP(varargin)
            HLP@high_level_planner(varargin{:}) ;
            HLP.plot_data.nodes = [] ;
        end
        
        %% plan a path
        function out = grow_tree(HLP,agent_info,obstacles)
            % grow tree
            start_tic = tic ;
            t_cur = toc(start_tic) ;
            
            if HLP.grow_new_tree_every_iteration_flag
                z = agent_info.position(:,end) ;
                nodes = [z] ;
                nodes_parent = [0] ;
                cost = [0] ;
            else
                nodes = HLP.current_nodes ;
                nodes_parent = HLP.current_nodes_parent ;
                cost = HLP.current_cost ;
            end
            
            B = HLP.bounds ;
            NNGD = HLP.new_node_growth_distance ;
            
            while t_cur < HLP.timeout
                if rand < HLP.final_goal_rate
                    % choose the final goal as a node to explore in some
                    % certain probability
                    rand_node = HLP.goal;
                else
                    % generate a random node to explore
                    rand_node = rand_range(B(1:2:end),B(2:2:end)) ;
                    rand_node = rand_node(:) ;
                end
                
                % find the nearest node in the tree
                node_distances = vecnorm(rand_node - nodes) ;
                [nearest_node_distance,nearest_node_index] = min(node_distances) ;
                
                % grow the tree towards the direction new node
                if nearest_node_distance <= NNGD
                    new_node = rand_node;
                else
                    nearest_node = nodes(:,nearest_node_index) ;
                    new_node_direction = rand_node - nearest_node ;
                    new_node_direction = new_node_direction / norm(new_node_direction) ;
                    new_node = nearest_node + NNGD.*new_node_direction ;
                end
                
                if HLP.node_feasibility_check(new_node,nodes(:,nearest_node_index),obstacles) == true
                    near_nodes = [];
                    near_nodes_distance = [];
                    near_distance = HLP.near_distance_rate*(log(size(nodes,2))/size(nodes,2))^(1/HLP.dimension);

                    for idx = 1:size(nodes,2)
                        node_distance = norm(nodes(:,idx)-new_node);
                        if node_distance < near_distance && HLP.node_feasibility_check(nodes(:,idx),new_node,obstacles) == true 
                            near_nodes = [near_nodes,idx];
                            near_nodes_distance = [near_nodes_distance,node_distance];
                        end
                    end
                    
                    % find the nearest node, grow the tree
                    min_node = nearest_node_index;
                    min_cost = cost(nearest_node_index) + norm(nodes(:,nearest_node_index)-new_node);
                    for idx = 1:size(near_nodes,2)
                        near_node_idx = near_nodes(idx);
                        if cost(near_node_idx)+near_nodes_distance(idx)<min_cost
                            min_node = near_node_idx;
                            min_cost = cost(near_node_idx)+near_nodes_distance(idx);
                        end
                    end
                    
                    nodes = [nodes,new_node];
                    nodes_parent = [nodes_parent,min_node];
                    
                    cost = [cost,min_cost];
                    new_node_index = size(nodes,2);
                    
                    % rewire the tree
                    for idx = 1:size(near_nodes,2)
                        near_node = near_nodes(idx);
                        if cost(new_node_index)+near_nodes_distance(idx)<cost(near_node)
                            nodes_parent(near_node) = new_node_index;
                        end
                    end
                end
                
                t_cur = toc(start_tic) ;
            end 
            
            traverse_index = -1;
            backup_index = -1;
            min_goal_cost = inf;
            min_goal_distance = inf;
            for idx = 1:size(nodes,2)
                node_distance = norm(nodes(:,idx)-HLP.goal);
                if node_distance < NNGD 
                    if cost(idx) < min_goal_cost
                        min_goal_cost = cost(idx);
                        traverse_index = idx;
                    end
                end
                
                if node_distance < min_goal_distance
                    min_goal_distance = node_distance;
                    backup_index = idx;
                end
                
                % FOR DEBUGGING
                % fprintf("%d. [%f %f], %d, %f\n",idx,nodes(1,idx),nodes(2,idx),nodes_parent(idx),cost(idx));
            end
            
            if traverse_index == -1
                traverse_index = backup_index;
                out = false;
            else
                out = true;
            end
            
            HLP.plan = [HLP.goal];
            while traverse_index > 0
                HLP.plan = [HLP.plan, nodes(:,traverse_index)];
                traverse_index = nodes_parent(traverse_index);
            end

            HLP.plan_index = size(HLP.plan,2);
            
            HLP.current_nodes = nodes ;
            HLP.current_nodes_parent = nodes_parent ;
            HLP.current_cost = cost ;
        end
        
        %% get waypoint
        function wp = get_waypoint(HLP, agent_info, world_info, lookahead_distance)
            % get the agent pose
            z = agent_info.position(:,end) ;
            
            % call the RRT star algorithm
            HLP.grow_tree(agent_info, world_info.obstacles)
            
            if HLP.plan_index > 0
                if norm(z-HLP.plan(:,HLP.plan_index)) > HLP.change_plan_distance
                    wp = HLP.plan(:,HLP.plan_index);
                else
                    HLP.plan_index = HLP.plan_index - 1;
                    if HLP.plan_index > 0
                        wp = HLP.plan(:,HLP.plan_index);
                    else
                        wp = HLP.goal;
                    end
                end
            else
                wp = HLP.goal;
            end
            
            HLP.current_waypoint = wp;
        end
        
        %% node feasibility check
        function out = node_feasibility_check(HLP,node_A,node_B,obstacles)
            % this method should return TRUE if the node is feasible
            
            X = [node_A, node_B] ;
            O = obstacles ;
            
            if isempty(O)
                out = true ;
            else
                [check_inside,~] = inpolygon(X(1,:)',X(2,:)',O(1,:)',O(2,:)') ;
                [check_boundary,~] = polyxpoly(X(1,:)',X(2,:)',O(1,:)',O(2,:)') ;
                out = isempty(check_boundary) & ~any(check_inside) ;
            end
        end
        
        %% plotting
        function plot(HLP)
            N = HLP.current_nodes ;
            if check_if_plot_is_available(HLP,'nodes')
                HLP.plot_data.nodes.XData = N(1,:) ;
                HLP.plot_data.nodes.YData = N(2,:) ;
            else
                if ~isempty(N)
                    HLP.plot_data.nodes = plot(N(1,:),N(2,:),'b.') ;
                end
            end
        end
    end
end