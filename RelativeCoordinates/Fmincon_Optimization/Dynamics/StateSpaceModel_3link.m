function [dx,q,dq,y,dy,u,pSwingFoot_end,GRF_iter,s] = StateSpaceModel_3link(t,x,pStanceFoot,q1Switch,h_alpha)
% StateSpaceModel_3links - computes dynamics of the 3 link model. dx can be
%   solved with ode45

%% Lagrange Model
dim_q = length(x)/2 ;
q = x(1:dim_q) ;
dq = x(dim_q+1:end) ;
[D,Cdq,G,B] = LagrangeModel_3link(x) ; % Compute Lagrange equation
                                           % coefficients
%% Compute Control Law
[u,y,dy,s] = Controller(x,q1Switch,h_alpha) ;
ddq = inv(D)*(B*u - Cdq - G) ; % Rearrange Lagrange equation and solve for
                               % ddq
dx = [dq ; ddq] ;              % time derivative of the state space

%% Ground Reaction Forces (GRF)
[De, Ce_dqe, Ge, Be] = LagrangeModelextend_3link(x) ;
E1 = [0 0 0 1 0 ; 0 0 0 0 1] ;
GRF_iter = GRF_3link(De,Ce_dqe,Ge,Be,E1,u) ;
    
%% Swing Foot Height
[pHip,pTorso,pSwingFoot_end] = Points_3link(x,pStanceFoot) ; % compute positions
end


% Check y=0, dy=0. And check if kp and kd are small to see that u is the
% same. Plot check. And check on the optimizer.

