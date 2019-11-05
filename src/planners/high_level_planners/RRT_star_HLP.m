classdef RRT_star_HLP < RRT_HLP
    properties
        cost
        rewire_distance = 1 ;
        best_path_cost ;
    end
    
    methods
        %% constructor
        function HLP = RRT_star_HLP(varargin)
            HLP@RRT_HLP(varargin{:}) ;
        end
        
        %% tree growth
        function initialize_tree(HLP,agent_info)
            initialize_tree@RRT_HLP(HLP,agent_info)
            
            % initialize cost
            switch HLP.grow_tree_mode
                case 'new'
                    HLP.cost = 0 ;
                case 'seed'
                    HLP.nodes = HLP.best_path_cost ;
                case 'keep'
                    HLP.vdisp('Keeping previous cost!',10)
                otherwise
                    error('Please pick a valid tree growth mode!')
            end
            
            if isempty(HLP.nodes)
                HLP.cost = 0 ;
            end
        end
        
        %% extend
        function extend_success_flag = extend(HLP,rand_node,obstacles)
            % superclass call
            extend_success_flag = extend@RRT_HLP(HLP,rand_node,obstacles) ;
            
            % rewire
            if extend_success_flag
                HLP.rewire(obstacles) ;
            end
        end
      
        
        %% add node
        function add_node_to_tree(HLP,new_node,parent_index)
            add_node_to_tree@RRT_HLP(HLP,new_node,parent_index)
            
            % update cost
            new_cost = HLP.cost(parent_index) + vecnorm(new_node - HLP.nodes(:,parent_index)) ;
            HLP.cost = [HLP.cost, new_cost] ;
        end
        
        function rewire(HLP,obstacles)
            % get the newest node
            new_node = HLP.nodes(:,end) ;
            
            % get indices of all nodes that are candidates for rewiring
            idxs = 1:(HLP.N_nodes-1) ; % ignore the latest node
            idxs(idxs == HLP.nodes_parent(end)) = [] ; % ignore the latest parent
            
            % find all nodes within rewire distance of new node
            near_distances = vecnorm(HLP.nodes(:,idxs) - repmat(HLP.nodes(:,end),1,HLP.N_nodes - 2)) ;
            near_distances_log = near_distances <= HLP.rewire_distance ;
            near_idxs = idxs(near_distances_log) ;
            
            % for each near node, check if its edge is feasible and its
            % cost is lower by attaching to the new node
            for idx = near_idxs
                near_node = HLP.nodes(:,idx) ;
                new_edge_feasible = HLP.edge_feasibility_check(new_node,near_node,obstacles) ;
                
                % if HLP.plot_while_growing_tree_flag
                %     plot_path(near_node,'r.')
                %     drawnow()
                % end
                
                if new_edge_feasible
                    new_cost = HLP.cost(end) + vecnorm(new_node - near_node) ;
                    if new_cost < HLP.cost(idx)
                        HLP.nodes_parent(idx) = HLP.N_nodes ;
                        
                        % if HLP.plot_while_growing_tree_flag
                        %     plot_path(near_node,'ro')
                        %     drawnow()
                        % end
                    end
                end
            end
        end
    end
end