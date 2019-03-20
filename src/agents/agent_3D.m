classdef agent_3D < agent
methods
    %% constructor
    function A = agent_3D(varargin)
        A@agent(varargin{:}) ;
        A.n_states = 3 ;
        A.position_indices = 1:3 ;
        A.plot_sensor_radius = false ;
        A.reset ;
    end
    
    %% reset
    function reset(A,state)
        % method: reset(state)
        %
        % Zeros out the agent's states and inputs, and sets its time to
        % zero. By default, the state input can be a point in R^3, i.e. a
        % point (x,y,z), or it can be A.n_states-by-1.
        if nargin < 2
            reset@agent(A) ;
        else
            reset@agent(A,state) ;
        end
    end
    
    %% plotting
    function plot(A,color)
        if nargin < 2
            color = [0 0 1] ;
        end
        
        % get data to plot
        xyz = A.state(A.position_indices,:) ;
        
        % check whether or not to create new plot data
        if A.check_if_plot_is_available('trajectory')
            A.vdisp('Updating plot',5) ;
            A.plot_data.trajectory.XData = xyz(1,:) ;
            A.plot_data.trajectory.YData = xyz(2,:) ;
            A.plot_data.trajectory.ZData = xyz(3,:) ;
        else
            A.vdisp('Plotting new data',5)
            
            % plot trajectory
            trajectory_data = plot3(xyz(1,:),xyz(2,:),xyz(3,:),'Color',color) ;
            
            % update plot data
            A.plot_data.trajectory = trajectory_data ;
        end
    end
end
end