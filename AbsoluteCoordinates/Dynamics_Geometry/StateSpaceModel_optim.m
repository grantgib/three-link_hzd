function [dx,q,dq,y,dy,u,pSwingFoot_end,GRF_iter,s] = StateSpaceModel_optim(x)
% StateSpaceModel_3links - computes dynamics of the 3 link model. dx can be
%   solved with ode45

%% Global Variables
GlobalVar

%% Lagrange Model
dim_q = length(x)/2 ;
q = x(1:dim_q) ;
dq = x(dim_q+1:end) ;
[D,Cdq,G,B] = LagrangeModel_optim(q, dq) ; % Compute Lagrange equation
                                           % coefficients
%% Compute Control Law
OptimCondition = false ;
[u,y,dy,s] = ControlTorques_Optim(x) ;
ddq = inv(D)*(B*u - Cdq - G) ; % Rearrange Lagrange equation and solve for
                               % ddq
dx = [dq ; ddq] ;              % time derivative of the state space

%% Ground Reaction Forces (GRF)
[De, Ce_dqe, Ge, Be] = LagrangeModelextend_optim(x) ;
E1 = [0 0 0 1 0 ; 0 0 0 0 1] ;
GRF_iter = GRF_optim(De,Ce_dqe,Ge,Be,E1,u) ;
    
%% Swing Foot Height
[pHip,pTorso,pSwingFoot_end] = Points_optim(x) ; % compute positions

end

