function [ J_opt, u_opt_ind ] = ValueIteration( P, G )
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space.

% put your code here

%%initialize variables

% get dimension of number of states and number of controls
tic
numStates=size(P,1);
numControls=size(G,2);

%% set up J_opt and J_update
% J_opt is the cost to go 
J_opt=zeros(numStates,1); 

%J_update is the newly calculated cost to go 
J_update=ones(numStates,1);

%u_opt_ind is the optimal policy for  number of states
u_opt_ind=zeros(numStates,1);
% error
err = 1e-7;

%% value update
while max(abs(J_update-J_opt))>err

	J_opt=J_update; 
    

    for i=1:numStates
    	temp_J=zeros(numControls,1); % temp_J is numControls*1 vector to store current J(i) for all policies

        for l=1:numControls
            
            temp_J(l)=temp_J(l)+P(i,:,l)*J_opt;  % compute sum_j( P(i,j,u)*J(j) ) at a given policy
  
        end 
	    [J_update(i),u_opt_ind(i)]=min(temp_J+G(i,:)'); % choose the minimum P_k+1(i) among all optimal policies
	end
end

toc
end