function [ J_opt, u_opt_ind ] = LinearProgramming( P, G )
%LINEARPROGRAMMING Value iteration
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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

tic
%%
numStates=size(G,1);
numControls=size(G,2);

u_opt_ind=zeros(numStates,1); 
J_opt=zeros(numStates,1);


A=[];
b=[];
f=-1*ones(numStates,1);


%% give value to A,b,f
for l=1:numControls
    A=[A;(eye(numStates)-P(:,:,l))];
    b=[b;G(:,l)];
end 


%We need to remove Inf rows of cost G because it's meaningless 
% and with Inf ,we can't solve the equation.

InfG_Index=[];
for i=1:numStates*numControls

	if b(i,1)==Inf
		InfG_Index=[InfG_Index;i];
	 end
	
end
b(InfG_Index,:)=[];
A(InfG_Index,:)=[];


%% matlab linpro to solve linear programming
J_opt= linprog(f,A,b);



%% given J_opt ,now we can find u_opt_ind

for i=1:numStates
    temp_J=zeros(numControls,1); %temp_J is a numControls*1 vector to store 
         
                            % current cost to go J(i) of last step in Bellman equation
	for l=1:numControls
	    
	    temp_J(l)=temp_J(l)+P(i,:,l)*J_opt;
	    
	end


	[~,u_opt_ind(i)]=min(temp_J+G(i,:)');
end

toc

end





