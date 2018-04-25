classdef planner2D < handle
    properties
    % generic planner properties
        name
        bounds % world bounds plus planner-specific buffer
        default_buffer % minimum amount to buffer obstacles, given by the world
        current_waypoint
        current_obstacles
        verbose
        timeout % time allowed for "replan" function to execute
        t_plan % same as timeout; just for notational purposes
        t_move % amount of time the planner expects the agent to move
        xy_plan % current anticipated trajectory
        info % information about trajectory planning
    end
    
    methods
        function P = planner2D(timeout,verbose_level)
            if nargin > 0
                P.timeout = timeout ;
                P.t_plan = timeout ;
            else
                P.timeout = 1 ;
                P.t_plan = 1 ;
            end
            
            if nargin > 1
                P.verbose = verbose_level ;
            else
                P.verbose = 0 ;
            end

            P.xy_plan = [] ;
            P.info = [] ;
        end
        
        function setup(~,~,~)
            warning('Planner setup function is undefined!')
        end
        
        function update(~,~,~)
            warning('Planner update function is undefined!')
        end
        
        function [Tout,Uout,Zout] = replan(~,~,~)
            warning('Planner replan function is undefined!')
            Tout = [] ; Uout = [] ; Zout = [] ;
        end
        
        function plotInLoop(P,n,c)
            P.vdisp('Plotting in loop',4)
            
            if nargin < 3
                c = [1 0 0] ;
            end
            
            figure(n) ;
            
            % plot anticipated trajectory from planner
            if ~isempty(P.xy_plan)
                plot(P.xy_plan(1,:),P.xy_plan(2,:),':','LineWidth',2,'Color',c)
            end
        end
        
        function plotResults(~,~,~,~)
            % plotResults(figure_number, summary, summary_index)
            %
            % This method does not need to do anything by default, since
            % the planners are working in the loop but don't usually save
            % info; this can be overwritten by subclasses to create a
            % separate plot or add to the current final plot if needed (for
            % example, if a planner creates many candidate plans over the
            % course of a simulation, this function can plot all of the
            % candidates at the end of the simulation).
        end
        
        function vdisp(P,s,l)
        % Display a string s if the message's verbose level l is greater
        % than or equal to the planner's verbose level.
            if nargin < 3
                l = 1 ;
            end
            if P.verbose >= l
                disp(['    P: ',s])
            end
        end
    end
end
