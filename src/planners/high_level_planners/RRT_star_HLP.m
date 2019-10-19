classdef RRT_star_HLP < high_level_planner
    properties
        timeout = 0.4 ; % seconds
        final_goal_rate = 0.08;
        near_distance_rate = 6;
        change_plan_distance = 0.6;
        plan = [];
        plan_index = 0;
        bounds
    end
    methods
        %% constructor
        function HLP = RRT_star_HLP(varargin)
            HLP@high_level_planner(varargin{:}) ;
        end
        
        %% plan a path
        function out = plan_path(HLP,agent_info,obstacles,lookahead_distance)
            % get agent's current position
            z = agent_info.position(:,end) ;
        
            % grow tree
            start_tic = tic ;
            t_cur = toc(start_tic) ;
            nodes = [z];
            nodes_father = [0];
            cost = [0];
            B = HLP.bounds;
            for idx = 1:HLP.dimension
                width = B(2*idx) - B(2*idx-1);
                B(2*idx-1) = B(2*idx-1) + width * 0.04;
                B(2*idx) = B(2*idx) - width * 0.04;
            end
            
            while t_cur < HLP.timeout
                if rand < HLP.final_goal_rate
                    % choose the final goal as a node to explore in some
                    % certain probability
                    rand_node = HLP.goal;
                else
                    % generate a random node to explore
                    rand_node = rand(HLP.dimension, 1);
                    for idx = 1:HLP.dimension
                        rand_node(idx) = rand_node(idx) * (B(2*idx) - B(2*idx-1)) + B(2*idx-1);
                    end
                end
                
                % find the nearest node in the tree
                nearest_node_index = 1;
                nearest_node_distance = inf;
                for idx = 1:size(nodes,2)
                    node_distance = norm(rand_node - nodes(:,idx));
                    if(node_distance < nearest_node_distance)
                        nearest_node_index = idx;
                        nearest_node_distance = node_distance;
                    end
                end
                
                % grow the tree towards the direction of this new node
                if nearest_node_distance <= lookahead_distance
                    new_node = rand_node;
                else
                    new_node = nodes(:,nearest_node_index) + lookahead_distance .* (rand_node - nodes(:,nearest_node_index)) ./ nearest_node_distance;
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
                    nodes_father = [nodes_father,min_node];
                    
                    cost = [cost,min_cost];
                    new_node_index = size(nodes,2);
                    
                    % rewire the tree
                    for idx = 1:size(near_nodes,2)
                        near_node = near_nodes(idx);
                        if cost(new_node_index)+near_nodes_distance(idx)<cost(near_node)
                            nodes_father(near_node) = new_node_index;
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
                if node_distance < lookahead_distance 
                    if cost(idx) < min_goal_cost
                        min_goal_cost = cost(idx);
                        traverse_index = idx;
                    end
                end
                
                if node_distance < min_goal_distance
                    min_goal_distance = node_distance;
                    backup_index = idx;
                end
                
                % fprintf("%d. [%f %f], %d, %f\n",idx,nodes(1,idx),nodes(2,idx),nodes_father(idx),cost(idx));
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
                traverse_index = nodes_father(traverse_index);
            end

            HLP.plan_index = size(HLP.plan,2);
        end
        
        %% get waypoint
        function wp = get_waypoint(HLP, agent_info, world_info, lookahead_distance)
            % get the agent pose
            z = agent_info.position(:,end) ;
            
            % call the RRT star algorithm
            HLP.plan_path(agent_info, world_info.obstacles, lookahead_distance) 
            
            
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
            out = true;
            
            N = size(obstacles,2)/6 ; % number of obstacles
            
            for idx = 1:6:(6*N-1)
                % judge if the nodes of the line is inside the obstacles
                if HLP.get_cross(obstacles(:,idx), obstacles(:,idx+1), node_A) * HLP.get_cross(obstacles(:,idx+2), obstacles(:,idx+3), node_A) >= 0 && HLP.get_cross(obstacles(:,idx+1), obstacles(:,idx+2), node_A) * HLP.get_cross(obstacles(:,idx+3), obstacles(:,idx), node_A) >= 0
                    out = false;
                    break;
                end
                
                if HLP.get_cross(obstacles(:,idx), obstacles(:,idx+1), node_B) * HLP.get_cross(obstacles(:,idx+2), obstacles(:,idx+3), node_B) >= 0 && HLP.get_cross(obstacles(:,idx+1), obstacles(:,idx+2), node_B) * HLP.get_cross(obstacles(:,idx+3), obstacles(:,idx), node_B) >= 0
                    out = false;
                    break;
                end
                
                % judge if the line is crossing the obstacles
                for i = idx:1:(idx+3)
                    if max(obstacles(1,i),obstacles(1,i+1))<min(node_A(1),node_B(1)) || max(obstacles(2,i),obstacles(2,i+1))<min(node_A(2),node_B(2)) || max(node_A(1),node_B(1))<min(obstacles(1,i),obstacles(1,i+1)) || max(node_A(2),node_B(2))<min(obstacles(2,i),obstacles(2,i+1))
                        continue;
                    else
                        if HLP.vec_cross(obstacles(:,i)-node_B,node_A-node_B) * HLP.vec_cross(obstacles(:,i+1)-node_B,node_A-node_B) <= 0 && HLP.vec_cross(node_A-obstacles(:,i+1),obstacles(:,i)-obstacles(:,i+1)) * HLP.vec_cross(node_B-obstacles(:,i+1),obstacles(:,i)-obstacles(:,i+1)) <= 0
                            out = false;
                            break;
                        end
                    end
                end
                
                if out == false
                    break;
                end
            end 
        end
        
        function out = get_cross(HLP, p1, p2, p) 
            out = (p2(1) - p1(1)) * (p(2) - p1(2)) - (p(1) - p1(1)) * (p2(2) - p1(2));
        end
        
        function out = vec_cross(HLP, p1, p2) 
            out = p1(1) * p2(2) - p2(1) * p1(2);
        end
    end
end