classdef mpc_agent < agent2D
    
properties
    reference_trajectory
    reference_input
    reference_index
    time_discretization
    prediction_horizon
    A_jacobian
    B_jacobian
    input_range
    linearized_xy_range
    linearized_heading_range
    n_decision_variables
    n_decision_variable_states
    n_decision_variable_inputs
end

methods
%% constructoer

function A=mpc_agent(varargin)
    
    
    A.n_decision_variables = (A.prediction_horizon+1) * A.n_states + A.prediction_horizon*A.n_inputs;
    A.n_decision_variable_states = (A.prediction_horizon+1) * A.n_states;
    A.n_decision_variable_inputs = A.n_decision_variables-A.n_decision_variable_inputs;
end
    


%% functions to get constraints for lmpc program        
function [Aeq,beq] = get_equality_constraints(A)
%build matrix for A_i*x_i+B_i*u_i-x_{i+1}=0
%in the form Aeq*z=beq
%initial_idx specifies the time index of initial condition from the reference trajectory 
%A and B are function handles above

%initial condition
x_initial = A.state(:,end) - A.nominal_state(:,A.reference_index);

Aeq = zeros(A.n_decision_variable_states,A.n_decision_variables);
Aeq(1:A.n_states,1:A.n_states) = eye(A.n_states); %initial condition 
beq = zeros(A.n_decision_variable_states,1);
beq(1:A.n_states) = x_initial;

state_idxs = A.n_states + 1:A.n_states:A.n_decision_variable_states;
input_idxs = A.n_decision_variable_states + 1:A.n_inputs:A.n_decision_variables;

for i=1:A.prediction_horizon
    
    ztemp=A.reference_trajectory(:,A.reference_index+i-1);
    
    utemp=A.reference_input(:,A.reference_index+i-1);
    
    %negative identity for i+1
    Aeq(state_idxs(i):state_idxs(i)+A.n_states-1,state_idxs(i):state_idxs(i)+A.n_states-1) = -eye(A.n_states);
    
    %A matrix for i
    Aeq(state_idxs(i):state_idxs(i)+A.n_states-1,state_idxs(i)-A.n_states:state_idxs(i)-1) = A.A_jacobian(ztemp,utemp);
    
    %B matrix for i
    Aeq(state_idxs(i):state_idxs(i)+A.n_states-1,input_idxs(i):input_idxs(i)+A.n_inputs-1) = A.B_jacobian(ztemp,utemp);
end

end
 
function [Aineq,bineq] = get_inequality_constraints(A)   
    Aineq_xy=[];
    Aineq_h=[];
    Aineq_input=[];
    
    bineq_xy=[];
    bineq_h=[];
    bineq_input=[];
    
    %get bounds for linearized xy state
    if ~isempty(A.linearized_xy_range)
        Aineq_xy = [get_state_selector_matrix(A.xy_state_indices);-get_state_selector_matrix(A.xy_state_indices)];
        bineq_xy = [repmat(A.linearized_xy_range(:,2),[A.prediction_horizon+1,1]);...
            -repmat(A.linearized_xy_range(:,1),[A.prediction_horizon+1,1])];
    end
    
    %get bounds for linearized heading state
    if ~isempty(A.linearized_heading_range)
        Aineq_h = [get_state_selector_matrix(A.heading_state_index);-get_state_selector_matrix(A.heading_state_index)];
        bineq_h = [repmat(A.linearized_heading_range(2),[A.prediction_horizon+1,1]);...
            -repmat(A.linearized_heading_range(:,1),[A.prediction_horizon+1,1])];
    end
    
    %get bounds for input (range is given for the system model (not
    %linearized)
    if ~isempty(A.input_range)
        Aineq_input = [get_input_selector_matrix(1:A.n_inputs);-get_input_selector_matrix(1:A.n_inputs)];
        bineq_input_ub = repmat(A.input_range(:,2),[1,A.prediction_horizon])-A.reference_input(A.reference_index:A.reference_index+A.prediction_horizon-1);
        bineq_input_lb = repmat(A.input_range(:,1),[1,A.prediction_horizon])-A.reference_input(A.reference_index:A.reference_index+A.prediction_horizon-1);
        bineq_input=[bineq_input_ub;-bineq_input_lb];
    end
    
    Aineq=[Aineq_xy;Aineq_h;Aineq_input];
    
    bineq=[bineq_xy;bineq_h;bineq_input];
   
    
end

%% helper functions

%generate jacobians for euler constraints
function [A_jac,B_jac] = generate_jacobians_from_symbolic_dynamics(A,symbolic_dynamics,z,u)
    A_jac_cont = jacobian(symbolic_dynamic,z);
    A_jac_discrete = eye(A.n_states)+A.time_discretization*A_jac_cont;
    B_jac
end

%select all states or inputs from decision variable
function [selector_matrix] = get_state_selector_matrix(A,state_indexs,number)
     if nargin<3
        number=A.prediction_horizon+1;
    end
    tmp=zeros(1,A.n_states);
    tmp(state_indexs)=1;
    tmp_repeated=repmat({tmp},number,1);
    
    selector_matrix=blkdiag(tmp_repeated{:});
    
    selector_matrix=[selector_matrix,zeros(number,A.n_decision_variable_inputs)];
end

function [selector_matrix] = get_input_selector_matrix(A,input_indexs,number)
    if nargin<3
        number=A.prediction_horizon;
    end
    tmp=zeros(1,A.n_inputs);
    tmp(input_indexs)=1;
    tmp_repeated=repmat({tmp},number,1);
    
    selector_matrix=blkdiag(tmp_repeated{:});
    
    selector_matrix=[zeros(number,A.n_decision_variable_states),selector_matrix];
end

end
    
end
