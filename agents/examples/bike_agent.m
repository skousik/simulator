classdef bike_agent<mpc_agent
    properties
        b
        L
    end
    methods
        
        function A= bike_agent(b,L,npred,dt,input_range)
      %cost for states
    Q=diag([1,1,0.5]);
    
    %cost for inputs
    R=diag([0.1,0.01]);
            
            syms t real;
z=sym('z',[3,1],'real');
u=sym('u',[2,1],'real');

syms t real;
z=sym('z',[3,1],'real');
u=sym('u',[2,1],'real');

symbolic_dynamics=[u(1)*cos(z(3))-b*u(1)/L*tan(u(2))*sin(z(3));...
    u(1)*sin(z(3))+b*u(1)/L*tan(u(2))*cos(z(3));...
    u(1)/L*tan(u(2))];

A@mpc_agent(symbolic_dynamics,t,z,u,'n_states',3,'n_inputs',2,'prediction_horizon',npred,...
    'time_discretization',dt,'input_range',input_range,'state_cost',Q,'input_cost',R)
                  A.b=b;
            A.L=L;
            
        end
        
function dzdt=dynamics(A,t,z,T,U,~)

if length(T)<=1 || isempty(T) || size(U,2)==1
    delta=U(2);
    u=U(1);
else
    delta=interp1(T',U(2,:)',t,'previous');
    u=interp1(T',U(1,:)',t,'previous');
end

dzdt=[u*cos(z(3))-A.b/A.L*u*tan(delta)*sin(z(3));...
      u*sin(z(3))+A.b/A.L*u*tan(delta)*cos(z(3));...
      u/A.L*tan(delta)];

end
end
end