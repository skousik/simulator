classdef segwayAgent < agent2D
% Class: segwayAgent < agent2D
%
% This agent models a dynamic unicycle, which replicates the behavior of a
% Segway RMP robot. In addition to the properties of the superclass
% agent2D, it has a maximum yaw rate (wmax), max velocity (vmax), max
% yaw accel (gmax), and max longitudinal accel (amax).

    properties
        wmax
        vmax
        amax
        gmax
        velocity_state_index
    end

    methods
        function A = segwayAgent(footprint,sensor_radius,wmax,vmax,...
                                 verbose_level)
            if ~exist('verbose_level','var')
                verbose_level = 0 ;
            end
            
            if nargin == 0
                footprint = 0.38 ;
                sensor_radius = 4 ;
                wmax = 1 ;
                vmax = 1.5 ;
                verbose_level = 0 ;
            end
            
            A@agent2D(verbose_level,...
                      'footprint',footprint,...
                      'sensor_radius',sensor_radius,...
                      'xy_state_indices',[1 2],...
                      'heading_state_index',3,...
                      'n_states',5,'n_inputs',2)
            
            if ~exist('wmax','var')
                wmax = 1 ;
            end
            
            if ~exist('vmax','var')
                vmax = 1.5 ;
            end
            
            % parameters from sys ID
            Kg = 2.95 ;
            Ka = 3.0 ;
            
            A.wmax = wmax ;
            A.vmax = vmax ;
            A.gmax = 2*Kg*wmax ;
            A.amax = Ka*vmax ;
            A.velocity_state_index = 5 ;
        end
        
        function reset(A,position)
            % starting pose is [x y h w v]^T
            switch size(position,1)
                case 2
                    A.state = [position(:) ; 0 ; 0 ; 0] ; 
                case 3
                    A.state = [position(:) ; 0 ; 0] ;
                case 5
                    A.state = position(:) ;
                otherwise
                    error('Invalid starting position given!')
            end
            A.time = 0 ;
            A.input = zeros(A.n_inputs,1) ;
            A.input_time = 0 ;
        end
        
        function zd = dynamics(A,t,z,T,U,Z)
            if nargin < 6
                Z = [] ;
            end
            
            % ZOH for inputs
            wdes = interp1(T,U(1,:),t,'previous') ; % yaw rate
            vdes = interp1(T,U(2,:),t,'previous') ; % velocity

            % extract the states
            h = z(3) ;
            w = z(4) ;
            v = z(5) ;

            % determine the inputs
            Kg = 2.95 ;
            Ka = 3.0 ;

            % if a trajectory to follow was provided, try to follow it
            if ~isempty(Z)
                htraj = interp1(T,Z(3,:),t) ;
                vtraj = interp1(T,Z(5,:),t) ;

                Kh = 10 ;
                Kv = 10 ;

                wdes = wdes + Kh*(htraj - h) ;
                vdes = vdes + Kv*(vtraj - v) ;
            end
            
            g = Kg*(wdes - w) ;
            a = Ka*(vdes - v) ;
            
            % saturate the inputs
            g = max(min(g,A.gmax),-A.gmax) ;
            a = max(min(a,A.amax),-A.amax) ;
            
            % calculate the derivatives
            xd = v*cos(h) ;
            yd = v*sin(h) ;
            hd = w ;
            wd = g ;
            vd = a ;

            % return state derivative
            zd = [xd ; yd ; hd ; wd ; vd] ;
        end
        
        function stop(A,tmax)
            if nargin < 2
                tmax = A.vmax/2 ;
            end
            T_input = [0, tmax] ;
            U_input = zeros(2,size(T_input,2)) ;
            A.move(T_input(end),T_input,U_input) ;
        end
        
        function plotInLoop(A,n,c)
            plotInLoop@agent2D(A,n,c) ;
            
            % plot robot heading
            h = A.state(A.heading_state_index,end) ;
            R = rotmat(h) ;
            zh = [0.1*cos(linspace(0,2*pi,4)) ;
                  0.075*sin(linspace(0,2*pi,4)) ] ;
            zh = R*zh + A.state(A.xy_state_indices,end) ;
            
            plot(zh(1,:),zh(2,:),'b-','LineWidth',1.5)            
        end
    end
end