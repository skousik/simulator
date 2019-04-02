classdef mpc_agent < agent_2D

properties
    reference_trajectory %store all reference trajecotries at A.input_time
    reference_input %store all reference inputs at A.input_time
    time_discretization %time discretization for mpc problem
    mpc_prediction_horizon % desired # of control inputs for mpc horizon
    A_jacobian %function returning the jacobian of mpc dynamic model wrt to state
    B_jacobian %function returning the jacobian of mpc dynamic model wrt to input
    input_range %n_inputs x 2 double for [min,max] of inputs allowed
    input_rate_range %n_inputs x 3 double for [min,max] of input rate allowed
    linearized_xy_range %2 x 2 double for [min,max] of xy error from reference
    linearized_heading_range %1 x 2 double for [min,max] of heading error from reference
    state_range %n_states x 2 doubles of [min,max] of state allowed
    state_cost %n_states x n_states double containing cost weights for states
    input_cost %n_inputs x n_inputs double containing cost weights for inputs
    agent_state_time_discretization %time discretization for A.time
    current_prediction_horizon % # of control inputs for current mpc problem
    %(is default until we reach end of reference trajectory)
    n_decision_variables % # of decision variables for current mpc problem
    n_decision_variable_states % # of decision variables that are states
    n_decision_variable_inputs % # of decision variables that are inputs
    initial_input% default initial inputs for rate constraints
end

methods
%% constructoer

function A=mpc_agent(mpc_dynamics,time_discretization,...
                     mpc_prediction_horizon,varargin)

    %input: mpc_dynamics is a function handle @(time,state,input). this
    %class will use the matlab symbolic toolbox to try to compute the
    %jacobians from the function. If there is an error a warning pops up.
    %and you have to set the jacobians manually

    A@agent_2D(varargin{:})

    for idx = 1:2:length(varargin)
        switch varargin{idx}
            case 'input_range'
                A.input_range = varargin{idx+1} ;
            case 'linearized_xy_range'
                A.linearized_xy_range = varargin{idx+1} ;
            case 'linearized_heading_range'
                A.linearized_heading_range = varargin{idx+1} ;
            case 'state_cost'
                A.state_cost = varargin{idx+1} ;
            case 'input_cost'
                A.input_cost = varargin{idx+1} ;
            case 'agent_state_time_discretization'
                A.agent_state_time_discretization = varargin{idx+1} ;
            case 'state_range'
                A.state_range = varargin{idx+1};
            case 'input_rate_range'
                A.input_rate_range = varargin{idx+1};
            case 'initial_input'
                A.initial_input = varargin{idx+1};
        end
    end

    A.time_discretization=time_discretization;

    A.mpc_prediction_horizon=mpc_prediction_horizon;

    if isempty(A.agent_state_time_discretization)
       A.agent_state_time_discretization=A.time_discretization;
       warning('setting agent state time discretization to mpc time discretization, this might affect the collision checks if too large')
    end


    %setup
    if A.agent_state_time_discretization>A.time_discretization
        A.agent_state_time_discretization=A.time_discretization;
    end

    try
        set_jacobians_from_mpc_dynamics_function(A,mpc_dynamics);
    catch
        warning('error when symbolically computing jacobians. youre gonna have to do this manually')
    end
end

%% function to reset states and reference trajectory
function reset(A,state,initial_input)
    % pad state with zeros
    if length(state) < A.n_states
        A.vdisp('Input to A.reset is too small! Padding with zeros.')
        state = [state ; zeros(A.n_states-size(state,1),1)] ;
    elseif length(state) > A.n_states
        error('Input is too large!')
    end

    % reset
    A.time = 0;
    A.state = state;
    A.input_time = 0;
    A.input =NaN(A.n_inputs,1);
    A.reference_trajectory = [];
    A.reference_input = [];
    
    if nargin>=3
        A.initial_input = initial_input;
    end
end


%% function to move and do mpc loop
function move(A,T_total,T_input,U_input,Z_desired)
    %if the desired time to move is longer than the reference trajf ectory,
    %throw an error
   
    
    if T_input(end) < T_total
        warning(['Provided input time vector is shorter than the ',...
            'desired motion time! The agent will only be ',...
            'moved until the largest available time in the ',...
            'input time vector.'])
        T_total = T_input(end) ;
    end


    if ~isempty(Z_desired)
        [T_out,Z_out,T_out_input,U_out,U_reference,Z_reference] = mpc_movement_loop(A,T_total,T_input,U_input,Z_desired);
    else
        [T_out,Z_out,T_out_input,U_out,U_reference,Z_reference] = open_loop_movement(A,T_total,T_input,U_input);
    end

    %store motion in agent structure

    store_movement(A,T_out,Z_out,T_out_input,U_out,U_reference,Z_reference);

end

%% store trajectories in agent function
function store_movement(A,T_out,Z_out,T_out_input,U_out,U_reference,Z_reference)
    
    Z_out(A.heading_state_index,:)=rad2heading(Z_out(A.heading_state_index,:));
    Z_reference(A.heading_state_index,:) = rad2heading(Z_reference(A.heading_state_index,:));

    A.state = [A.state(:,1:end-1),Z_out];

    A.time = [A.time,A.time(end)+T_out(2:end)];


    A.input = [A.input(:,1:end-1),U_out];

    A.input_time = [A.input_time , A.input_time(end)+T_out_input(2:end)];

    A.reference_input = [A.reference_input(:,1:end-1) ,U_reference];

    A.reference_trajectory = [A.reference_trajectory(:,1:end-1) , Z_reference];
    
 
end

%% function called when we stop (apply 0 input, no mpc)
function stop(A,t)

    %default is to apply 0 input to the dynamics for time t

    [ttemp,ztemp]=ode45(@(t,z)A.dynamics(t,z,0,zeros(A.n_inputs,1)),[0,t],A.state(:,end));

    T_out = uniquetol([0:A.agent_state_time_discretization:t,t]);

    T_out_input = uniquetol([0:A.time_discretization:t,t]);

    Z_out = interp1(ttemp,ztemp,T_out','pchip','extrap')';

    U_out = zeros(A.n_inputs,length(T_out_input));

    U_reference = NaN(size(U_out));

    Z_reference = NaN(A.n_states,length(T_out_input));

    store_movement(A,T_out,Z_out,T_out_input,U_out,U_reference,Z_reference);
   
end

%% functions to get constraints for lmpc program
function [Aeq,beq] = get_equality_constraints(A,x_initial,T,Z,U,reference_index)
%build matrix for A_i*x_i+B_i*u_i-x_{i+1}=0
%in the form Aeq*z=beq
%initial_idx specifies the time index of initial condition from the reference trajectory
%A and B are function handles above


Aeq = zeros(A.n_decision_variable_states,A.n_decision_variables);
Aeq(1:A.n_states,1:A.n_states) = eye(A.n_states); %initial condition
beq = zeros(A.n_decision_variable_states,1);
beq(1:A.n_states) = x_initial;

state_idxs = A.n_states + 1:A.n_states:A.n_decision_variable_states;
input_idxs = A.n_decision_variable_states + 1:A.n_inputs:A.n_decision_variables;

for i=1:A.current_prediction_horizon

    ztemp = Z(:,reference_index+i-1);

    utemp = U(:,reference_index+i-1);

    ttemp = T(:,reference_index+i-1);

    %negative identity for i+1
    Aeq(state_idxs(i):state_idxs(i)+A.n_states-1,state_idxs(i):state_idxs(i)+A.n_states-1) = -eye(A.n_states);

    %A matrix for i
    Aj = A.A_jacobian(ttemp,ztemp,utemp);
    if any(isnan(Aj))
        warning('setting NaNs in A_jacobian to 0')
        Aj(isnan(Aj))=0;
    end
        
    
    Aeq(state_idxs(i):state_idxs(i)+A.n_states-1,state_idxs(i)-A.n_states:state_idxs(i)-1) = Aj;

    %B matrix for i
    Bj =A.B_jacobian(ttemp,ztemp,utemp);
    if any(isnan(Bj))
        warning('setting NaNs in B_jacobian to 0')
        Bj(isnan(Bj))=0;
    end
    
    Aeq(state_idxs(i):state_idxs(i)+A.n_states-1,input_idxs(i):input_idxs(i)+A.n_inputs-1) = Bj;
end

end

function [Aineq,bineq] = get_inequality_constraints(A,x_initial,u_initial,~,Z,U,reference_index)
    Aineq=[];
    bineq=[];
    
    %get bounds for linearized xy state
    if ~isempty(A.linearized_xy_range)
        for xy_idx=1:2
        if (x_initial(A.xy_state_indices(xy_idx)) < A.linearized_xy_range(xy_idx,2)) 
            [Aineq,bineq]=add_state_bound(A,Aineq,bineq,A.xy_state_indices(xy_idx),A.linearized_xy_range(xy_idx,2),0);
        end
        if (x_initial(A.xy_state_indices(xy_idx)) >  A.linearized_xy_range(xy_idx,1))
            [Aineq,bineq]=add_state_bound(A,Aineq,bineq,A.xy_state_indices(xy_idx),A.linearized_xy_range(xy_idx,1),1);
        end
        end
    end

    %get bounds for linearized heading state
    if ~isempty(A.linearized_heading_range)
        if (x_initial(A.heading_state_index) < A.linearized_heading_range(2)) 
            [Aineq,bineq]=add_state_bound(A,Aineq,bineq,A.heading_state_index,A.linearized_heading_range(2),0);
        end
        if (x_initial(A.heading_state_index) > A.linearized_heading_range(1)) 
            [Aineq,bineq]=add_state_bound(A,Aineq,bineq,A.heading_state_index,A.linearized_heading_range(1),1);
        end
    end
    
    %get bounds for state range 
    if ~isempty(A.state_range)
       for state_idx=1:A.n_states
          if A.state_range(state_idx)>-Inf && x_initial(state_idx)+Z(state_idx,reference_index)>A.state_range(state_idx,1)
             [Aineq,bineq]=add_state_bound(A,Aineq,bineq,state_idx,A.state_range(state_idx,1),1,Z,reference_index);
          end
           if A.state_range(state_idx,2)<Inf && x_initial(state_idx)+Z(state_idx,reference_index)<A.state_range(state_idx,2)
             [Aineq,bineq]=add_state_bound(A,Aineq,bineq,state_idx,A.state_range(state_idx,2),0,Z,reference_index);
          end
       end
    end
    
    %add input rate bounds
    if ~isempty(A.input_rate_range)
        for input_idx=1:A.n_inputs
           
           if A.input_rate_range(input_idx,1)>-Inf
               [Aineq,bineq] = add_input_rate_bound(A,Aineq,bineq,input_idx,A.input_rate_range(input_idx,1),1,U,reference_index);
%               
                tmp = zeros(1,A.n_inputs);
                tmp(input_idx)=-1;
                Aineq=[Aineq;[zeros(1,A.n_decision_variable_states),tmp,zeros(1,A.n_decision_variable_inputs-A.n_inputs)]];
                 bineq=[bineq;-(A.input_rate_range(input_idx,1)*A.time_discretization+u_initial(input_idx))];
%                    
           end
            if A.input_rate_range(input_idx,2)<Inf
               [Aineq,bineq] = add_input_rate_bound(A,Aineq,bineq,input_idx,A.input_rate_range(input_idx,2),0,U,reference_index);
                      tmp = zeros(1,A.n_inputs);
                tmp(input_idx)=1;
                 Aineq=[Aineq;[zeros(1,A.n_decision_variable_states),tmp,zeros(1,A.n_decision_variable_inputs-A.n_inputs)]];
                 bineq=[bineq;A.input_rate_range(input_idx,2)*A.time_discretization+u_initial(input_idx)];
           end
        end
    end
    %get bounds for input (range is given for the system model (not
    %linearized)
    if ~isempty(A.input_range)

        Aineq_input = [zeros(A.n_decision_variable_inputs,A.n_decision_variable_states),eye(A.n_decision_variable_inputs);...
            zeros(A.n_decision_variable_inputs,A.n_decision_variable_states),-eye(A.n_decision_variable_inputs)];

        bineq_input_ub = repmat(A.input_range(:,2),[1,A.current_prediction_horizon])-U(:,reference_index:reference_index+A.current_prediction_horizon-1);
        bineq_input_lb = repmat(A.input_range(:,1),[1,A.current_prediction_horizon])-U(:,reference_index:reference_index+A.current_prediction_horizon-1);

        bineq_input_ub=reshape(bineq_input_ub,[A.n_inputs*A.current_prediction_horizon,1]);
        bineq_input_lb=reshape(bineq_input_lb,[A.n_inputs*A.current_prediction_horizon,1]);

        bineq_input=[bineq_input_ub;-bineq_input_lb];
    end

    Aineq=[Aineq;Aineq_input];

    bineq=[bineq;bineq_input];


end


%% helper functions
%set sizes for other functions
function set_problem_size(A,prediction_horizon)
    A.n_decision_variable_states=(prediction_horizon+1)*A.n_states;
    A.n_decision_variable_inputs=prediction_horizon*A.n_inputs;
    A.n_decision_variables=A.n_decision_variable_states+A.n_decision_variable_inputs;
    A.current_prediction_horizon=prediction_horizon;
end

%construct cost matrix
function H = get_cost_matrix(A)
    if isempty(A.state_cost)
        Q = eye(A.n_states);
    else
        Q = A.state_cost;
    end

    if isempty(A.input_cost)
        R = eye(A.n_inputs);
    else
        R = A.input_cost;
    end

    tmp_state=repmat({Q},A.current_prediction_horizon+1,1);
    tmp_input=repmat({R},A.current_prediction_horizon,1);
    tmp=[tmp_state;tmp_input];

    H=blkdiag(tmp{:});
end

%generate jacobians for euler constraints
function set_jacobians_from_mpc_dynamics_function(A,mpc_dynamics)
    syms t real;
    z=sym('z',[A.n_states,1],'real');
    u=sym('u',[A.n_inputs,1],'real');

    symbolic_dynamics=mpc_dynamics(t,z,u);

    A_jac_cont = jacobian(symbolic_dynamics,z');
    A_jac_discrete = eye(A.n_states)+A.time_discretization*A_jac_cont;
    A.A_jacobian = matlabFunction(A_jac_discrete,'Vars',{t,z,u});

    B_jac_cont = jacobian(symbolic_dynamics,u');
    B_jac_discrete = A.time_discretization*B_jac_cont;
    A.B_jacobian = matlabFunction(B_jac_discrete,'Vars',{t,z,u});
end

function [T_out,Z_out,T_out_input,U_out,U_reference,Z_reference] = open_loop_movement(A,T_total,T_input,U_input)


    T_out_input=uniquetol([0:A.time_discretization:T_total,T_total]);


    T_out=uniquetol([T_out_input,0:A.agent_state_time_discretization:T_total,T_total]);


    %simulate dynamics
    [~,Z_out]=ode45(@(t,z)A.dynamics(t,z,T_input,U_input),T_out,A.state(:,end));

    Z_out = Z_out';

    %interpolate reference trajectories to input timestep

    U_reference = interp1(T_input',U_input',T_out_input','previous','extrap')';
    U_out = U_reference;

    Z_reference = NaN(A.n_states,length(T_out_input));
end

function [T_out,Z_out,T_out_input,U_out,U_reference,Z_reference] = mpc_movement_loop(A,T_total,T_input,U_input,Z_desired)

    ref_time=uniquetol([T_input(1),T_input(1):A.time_discretization:T_input(end),T_input(end)]);
    
    %if the reference trajectory is not given at the proper time
    %discretization interporlate

    if any(diff(T_input)~=A.time_discretization)
        warning('reference trajectory not given at correct timestep')

        ref_time = uniquetol([T_input(1):A.time_discretization:T_input(end),T_input(end)]);
        U_input = interp1(T_input',U_input',ref_time','pchip','extrap')';
        Z_desired = interp1(T_input',Z_desired',ref_time','pchip','extrap')';
        T_input = ref_time;

    end

    %pre allocate vectors
    T_out_input=uniquetol([0:A.time_discretization:T_total,T_total]);
        
    U_out=NaN(A.n_inputs,length(T_out_input));

    T_out=uniquetol([T_out_input,0:A.agent_state_time_discretization:T_total,T_total]);

    Z_out=NaN(A.n_states,length(T_out));

    Z_out(:,1)=A.state(:,end);

    %mpc loop
    time_diff_matrix = T_out_input-T_out';
    
    for i=1:length(T_out_input)-1

        pred_horizon=min([A.mpc_prediction_horizon,length(T_input)-i]);

        set_problem_size(A,pred_horizon);

        [~,start_idx]=min(abs(time_diff_matrix(:,i)));
        [~,end_idx]=min(abs(time_diff_matrix(:,i+1)));

        tvec=T_out(start_idx:end_idx);

        x_initial=Z_out(:,start_idx)-Z_desired(:,i);

        x_initial(A.heading_state_index)=rad2heading(x_initial(A.heading_state_index));
        
        if i<2
           if size(A.input,2)>=2
               u_initial= A.input(:,end-1)-U_input(:,i);
           elseif~isempty(A.initial_input)
               u_initial = A.initial_input-U_input(:,i);
           else
              u_initial = zeros(A.n_inputs,1); 
           end
        else
           u_initial=U_out(:,i-1)-U_input(:,i);
        end

        [Aeq,beq] = get_equality_constraints(A,x_initial,T_input,Z_desired,U_input,i);

        [Aineq,bineq] = get_inequality_constraints(A,x_initial,u_initial,T_input,Z_desired,U_input,i);

        H = get_cost_matrix(A);

        f=zeros(A.n_decision_variables,1);
        
        try

        [x,~,exitflag] = quadprog(H,f,Aineq,bineq,Aeq,beq);
        catch
             warning('qp errored, apply reference input')
            x=zeros(A.n_decision_variables,1);
        end

        if exitflag<0 ||isempty(x)
            warning('qp infeasible, apply reference input')
            x=zeros(A.n_decision_variables,1);
        end
        %get linearized output
        u_mpc = x(A.n_decision_variable_states+1:A.n_decision_variable_states+A.n_inputs);

        U_out(:,i)=u_mpc+U_input(:,i);
        
        %double check it satisfies any input constraints
        if ~isempty(A.input_range)
            for j=1:A.n_inputs
                if(U_out(j,i)>A.input_range(j,2))
                    U_out(j,i)=A.input_range(j,2);
                end
                if(U_out(j,i)<A.input_range(j,1))
                    U_out(j,i)=A.input_range(j,1);
                end
            end
        end
        
        if ~isempty(A.input_rate_range)
            for j=1:A.n_inputs
                if(U_out(j,i)-u_initial(j)-U_input(j,i))/A.time_discretization>A.input_rate_range(j,2)
                    U_out(j,i)=A.input_rate_range(j,2)*A.time_discretization+u_initial(j)+U_input(j,i);
                end
                if(U_out(j,i)-u_initial(j)-U_input(j,i))/A.time_discretization<A.input_rate_range(j,1)
                    U_out(j,i)=A.input_rate_range(j,1)*A.time_discretization+u_initial(j)+U_input(j,i);
                end
            end
        end
        

        %simulate dynamics
        [~,ztemp]=ode45(@(t,z)A.dynamics(t,z,T_out_input(i),U_out(:,i)),tvec,Z_out(:,start_idx));

        if length(tvec)==2
            Z_out(:,start_idx:end_idx)=ztemp([1,end],:)';
        else
            Z_out(:,start_idx:end_idx)=ztemp';
        end

    end

    %interpolate reference trajectories to input timestep

    U_reference = interp1(T_input',U_input',T_out_input','pchip')';

    Z_reference = interp1(T_input',Z_desired',T_out_input','pchip')';
end


end

end

%% helper functions



%% matrix operations

%select all states or inputs from decision variable
function [Aineq_out,bineq_out] = add_state_bound(A,Aineq_in,bineq_in,state_index,bound,bound_type,Z,reference_index)
if strcmp(bound_type,'upper')
    bound_type = 0;
elseif strcmp(bound_type,'lower')
    bound_type = 1;
end

Aineq_out=[Aineq_in;(-1)^bound_type*get_state_selector_matrix(A,state_index)];
 
if nargin < 7
   %apply bound as if state is linearized 
   bineq_out=[bineq_in;(-1)^bound_type*repmat(bound,[A.current_prediction_horizon+1,1])];
else
   bineq_out=[bineq_in;...
       (-1)^bound_type*(repmat(bound,[1,A.current_prediction_horizon+1])-Z(state_index,reference_index:reference_index+A.current_prediction_horizon))'];
end

end

function [Aineq_out,bineq_out] = add_input_rate_bound(A,Aineq_in,bineq_in,input_index,bound,bound_type,U,reference_index)
if strcmp(bound_type,'upper')
    bound_type = 0;
elseif strcmp(bound_type,'lower')
    bound_type = 1;
end

Aineq_out=[Aineq_in;(-1)^bound_type*get_input_rate_selector_matrix(A,input_index)];

bineq_out=[bineq_in;...
    (-1)^bound_type*(repmat(bound*A.time_discretization,[1,A.current_prediction_horizon-1])-diff(U(input_index,reference_index:reference_index+A.current_prediction_horizon-1)))'];

end


function [selector_matrix] = get_state_selector_matrix(A,state_index,number)

    if nargin<3
        number=A.current_prediction_horizon+1;
    end

    tmp=zeros(1,A.n_states);
    tmp(state_index)=1;


    tmp_repeated=repmat({tmp},number,1);

    selector_matrix=blkdiag(tmp_repeated{:});

    selector_matrix=[selector_matrix,zeros(number,A.n_decision_variable_inputs)];
end

function [selector_matrix] = get_input_rate_selector_matrix(A,input_index,number)
    if nargin<3
        number=A.current_prediction_horizon-1;
    end
    tmp=zeros(1,A.n_inputs);

    tmp(1,input_index)=1;
    
    tmp_repeated=repmat({tmp},number,1);

    selector_matrix_1=[-blkdiag(tmp_repeated{:}),zeros(number,A.n_inputs)];
    
    selector_matrix_2=[zeros(number,A.n_inputs),blkdiag(tmp_repeated{:})];

    selector_matrix=[zeros(number,A.n_decision_variable_states),selector_matrix_1+selector_matrix_2];
end