classdef box_obstacle < obstacle
    properties
        center = [0;0];
        side_lengths = [1;1] ;
        plot_patch_data
        collision_check_patch_data % structure with faces and vertices
        plot_edge_width = 1.5 ;
    end
    
    methods
        %% constructor
        function O = box_obstacle(varargin)
            % parse input arguments
            O = parse_args(O,varargin{:}) ;
            
            % set dimension
            O.dimension = length(O.side_lengths) ;
            
            % create info for plotting and collision checking
            O.create_plot_patch_data ;
            O.create_collision_check_patch_data ;
            O.plot_data.body = [] ;
        end
        
        %% plot and collision check setup
        function create_plot_patch_data(O)
            l = O.side_lengths ;
            c = O.center ;
            
            switch O.dimension
                case 2
                    [F,V] = make_box(l,c) ;
                case 3
                    [F,V] = make_cuboid_for_patch(l(1),l(2),l(3),c) ;
            end
            
            O.plot_patch_data.faces = F ;
            O.plot_patch_data.vertices = V ;
        end
        
        function create_collision_check_patch_data(O)
            O.create_plot_patch_data() ;
            
            switch O.dimension
                case 2
                    O.collision_check_patch_data.faces = O.plot_patch_data.faces ;
                    O.collision_check_patch_data.vertices = O.plot_patch_data.vertices ;
                case 3
                    V = O.plot_patch_data.vertices ;
                    F = convhull(V) ;
                    O.collision_check_patch_data.faces = F ;
                    O.collision_check_patch_data.vertices = V ;
            end
        end
        
        %% plotting
        function plot(O)
            % get info for plotting body
            F = O.plot_patch_data.faces ;
            V = O.plot_patch_data.vertices ;
            
            if check_if_plot_is_available(O,'body')
                O.plot_data.body.Faces = F ;
                O.plot_data.body.Vertices = V ;
            else
                patch_data = patch('Faces',F,'Vertices',V,...
                                   'FaceAlpha',O.plot_face_opacity,...
                                   'FaceColor',O.plot_face_color,...
                                   'EdgeColor',O.plot_edge_color,...
                                   'EdgeAlpha',O.plot_edge_opacity,...
                                   'LineWidth',O.plot_edge_width) ;
                O.plot_data.body = patch_data ;
            end
        end
        
        function plot_at_time(O,~)
            O.plot() ;
        end
    end
end