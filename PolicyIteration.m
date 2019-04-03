function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Value iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
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
tic

%% initialize variables
numStates=size(P,1);
numControls=size(P,3);
err=1e-5;
%%
A=zeros(numStates);   %A and b are needed to solve linear equation
I=eye(numStates);
b=zeros(numStates,1);
%%
J_opt=zeros(numStates,1);
J_update=ones(numStates,1);
u_opt_ind=5*ones(numStates,1);


%%
while 1
    
%%construct P to solve J_update.If we know policy ,we can solve J

  for i=1:numStates 

    A(i,:)=I(i,:)-P(i,:,u_opt_ind(i)); 
    b(i)=G(i,u_opt_ind(i));

  end
  J_update=A\b;  % given policy u ,we can solve the linear equation to get J

%% quit when satisfy condition
  if max(abs(J_update-J_opt))<err
   
     break;
  else

      J_opt=J_update;  % update cost to go

   end



%% use J_K to update u_k+1  stage2

for i=1:numStates

    temp_J=zeros(numControls,1); %temp_J is a number of controls  dimension vector
                                 %which to store temporary Cost to go for
                                 %all policies
    for l=1:numControls

       
        temp_J(l)=temp_J(l)+P(i,:,l)*J_opt;%temp_J(l) is cost to go of last
                                                  %step 
        
    end

    [~,u_opt_ind(i)]=min(temp_J+G(i,:)');        % find the current optimal policy 
                                                  % update U_k to U_k+1
end %end for


end  %end while

toc
end   %end function
