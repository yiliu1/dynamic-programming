%ComputeStageCosts.m 
function G = ComputeStageCosts( stateSpace, controlSpace, map, gate, mansion, cameras )
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, controlSpace, map, gate, mansion,
%   cameras) computes the stage costs for all states in the state space
%   for all control inputs.
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
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.

% put your code here

p=ComputeTransitionProbabilities(stateSpace, controlSpace, map, gate, mansion, cameras );

% now  p is a k*K*L matrix represents the probability

K=size(p,1);
L=size(p,3);

G=Inf*ones(K,L);

for i =1:K
    for l=1:L


        newstate=new(stateSpace,i,l);
        [~,j]=ismember(newstate,stateSpace,'rows');  

        if j~=0

            x=newstate(1);
            y=newstate(2);
            if map(y,x)<0 & l~=5

                c=caught(newstate,cameras,map);

                G(i,l)=(1-(1-c)^4)*6+4;
            elseif map(y,x)==0 & l~=5

                c=caught(newstate,cameras,map);

                G(i,l)=1+6*c;

            elseif l==5

                t=picturetaken(newstate,mansion,map);

                c=caught(newstate,cameras,map);
               
                G(i,l)=1+(1-t)*c*6;



            end
              

        else

            
        end

    end
end


end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%function new=new(stateSpace,i,l)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function new=new(stateSpace,i,l)

if l==1
    new=stateSpace(i,:)+[0,1];
elseif l==2
    new=stateSpace(i,:)+[-1,0];
elseif l==3
    new=stateSpace(i,:)+[0,-1];
elseif l==4
    new=stateSpace(i,:)+[1,0];
else
    new=stateSpace(i,:);

end


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
                  break
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




