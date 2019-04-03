

function P = ComputeTransitionProbabilities(stateSpace, controlSpace, map, gate, mansion, cameras )

%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, controlSpace,
%   map, gate, mansion, cameras) computes the transition probabilities
%   between all states in the state space for all control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 2)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       controlSpace:
%           A (L x 1)-matrix, where the l-th row represents the l-th
%           element of the control space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map.
%           Positive values indicate cells that are inaccessible (e.g.
%           trees, bushes or the mansion) and negative values indicate
%           ponds or pools.
%
%   	gate:
%          	A (1 x 2)-matrix describing the position of the gate.
%
%    	mansion:
%          	A (F x 2)-matrix indicating the position of the cells of the
%           mansion.
%
%    	cameras:
%          	A (H x 3)-matrix indicating the positions and quality of the 
%           cameras.
%
%   Output arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

% put your code here



%  Reminder: controlSpace = [ 'n'; 'w'; 's'; 'e'; 'p' ]
       


k=size(stateSpace,1);
L=size(controlSpace,1);
P=zeros(k,k,L);
[~,gateposition]=ismember(gate,stateSpace,'rows');


for i=1:k

	for l=1:L

		if(l==1)
		    newstate=stateSpace(i,:)+[0,1];
		    P=transitionP(i,newstate,l,P,stateSpace, controlSpace, map, gate, mansion, cameras);
		elseif(l==2)
		    newstate=stateSpace(i,:)+[-1,0];
            P=transitionP(i,newstate,l,P,stateSpace, controlSpace, map, gate, mansion, cameras);
		    
		elseif(l==3)
		    newstate=stateSpace(i,:)+[0,-1];
            P=transitionP(i,newstate,l,P,stateSpace, controlSpace, map, gate, mansion, cameras);
        elseif(l==4)
            newstate=stateSpace(i,:)+[1,0];
            P=transitionP(i,newstate,l,P,stateSpace, controlSpace, map, gate, mansion, cameras);
        else

            newstate=stateSpace(i,:)+[0,0];
		    [~,j]=ismember(newstate,stateSpace,'rows'); 

		    t=picturetaken(newstate,mansion,map) ; %probability successfully take a picture 
		    c=caught(stateSpace(j,:),cameras,map);  %probability  caught
		    if (i==gateposition)

		    	P(i,i,l)=1-t;
		    else

		        P(i,gateposition,l)=(1-t)*c;
		        P(i,i,l)=(1-t)*(1-c);                 % need another terminal state
		    end
		end
	end
end



% end function ComputeTransitionProbabilities

end






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   below computes the transition probabolity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




function tr=transitionP(i,newstate,l,P,stateSpace, controlSpace, map, gate, mansion, cameras)

[~,gateposition]=ismember(gate,stateSpace,'rows');
[~,j]=ismember(newstate,stateSpace,'rows');


            if j>0 

		    	x=newstate(1);
		        y=newstate(2);
                
                if newstate==gate

                	P(i,j,l)=1;

                elseif map(y,x)==0 
                	P(i,j,l)=1-caught(stateSpace(j,:),cameras,map);
		    	    P(i,gateposition,l)=1-P(i,j,l);
		    	

		    	elseif map(y,x)<0

		    		c=caught(stateSpace(j,:),cameras,map);
		    	    P(i,j,l)=(1-c)^4;    %global variable pool_num_time_steps
		    	    P(i,gateposition,l)=1-P(i,j,l);
		    	end

		    else


		    	if i==gateposition

		    		P(i,i,l)=1;

          elseif map(stateSpace(i,2),stateSpace(i,1))<0     %%if i is in the pools


                 c=caught(stateSpace(i,:),cameras,map);
                 P(i,gateposition,l)=1-(1-c)^4;
                 P(i,i,l)=(1-c)^4;


		    	else

		 
		    	

                  
                   c=caught(stateSpace(i,:),cameras,map);

                
                   P(i,gateposition,l)=c;


		    	         P(i,i,l)=1-c;
		          
		        end

		    end


	tr=P;





end

            



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% now we give the function which calculate the probability caught by cameras
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function c=caught(state,cameras,map)
    
    c=size(cameras,1);
    p=zeros(c,1);
    
    for i=1:c
        
        if (state(1)==cameras(i,1))
            
            n=abs(cameras(i,2)-state(2));
            x=state(1);
            y=min(state(2),cameras(i,2));
            
            for j=1:n-1
                if map(y+j,x)>0
                    p(i)=-1;
                    break;
                end
            end
            
            if(p(i)==-1)
                p(i)=-1;
            else
                p(i)=cameras(i,3)/n;
            end
            
            
            
        elseif (state(2)==cameras(i,2))
            
            n=abs(cameras(i,1)-state(1));
            y=state(2);
            x=min(state(1),cameras(i,1));
            
            for j=1:n-1
                if map(y,x+j)>0
                    p(i)=-1;
                    break;
                end
            end
            
            if(p(i)==-1)
                p(i)=-1;
            else
                p(i)=cameras(i,3)/n;
            end
            
            
        else
            p(i)=-1;
            
            
        end
            
            
   
    end


    % p is the probability caught
    %1-p is the probability not caught 
    %c is probability not cauhgt by all cameras,which is a successul move

    
    a=p(p>0);
    
    if isempty(a)
        c=0;
    else

       c=1-prod(1-a);
   end
    
    
    
end  %end caught function




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% here is the function which calculate the probability successfully taken a picture
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function t=picturetaken(state,mansion,map)

n=size(mansion,1);

d=zeros(n,1);


for i =1:n

	if (state(1)==mansion(i,1))

		    m=abs(mansion(i,2)-state(2));
            x=state(1);
            y=min(state(2),mansion(i,2));

            for j=1:m-1
                if map(y+j,x)>0

                    d(i)=-1;
                    break;
                end
            end

            if d(i)~=-1

            	d(i)=m;
            end

    elseif (state(2)==mansion(i,2))

           m=abs(mansion(i,1)-state(1));
           y=state(2);
           x=min(state(1),mansion(i,1));

           for j=1:m-1
           	   if map(y,x+j)>0

           	   	  d(i)=-1;
           	   	  break;
           	   	end
           	end

           	if d(i)~=-1

           		d(i)=m;

           	end

    end



end


a=d>0;


if (isempty(a))
  t=0.001;
else
  b=d(d>0);
  mi=min(b);
  t=max([0.001,0.5/mi]);
  
end



end










