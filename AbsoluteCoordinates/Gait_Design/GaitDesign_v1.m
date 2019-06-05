% GaitDesign_v1 - use optimization techniques to optimize the 3-link
% walker's gait
%
% August 2018
%
clear
clear global
close all
addpath(genpath('Bezier_Files'))
addpath(genpath('Controller'))
addpath(genpath('Dynamics_Geometry'))
addpath(genpath('Impact_Poincare'))
addpath(genpath('Plots'))

%% Global Variables
GlobalVar
global Cost Geq Gineq x0 alphas
global pStanceFoot
global kp kd OptimCondition
global dimq
dimq = 3 ;
%% Initialize some other variables for fmincon
qi = [7*pi/8 - pi/6 ; 9*pi/8 - pi/6 ; pi/6] ;
dqi = [1.6 ; -1.6 ; 0] ;
alpha = [0.5 ; 0.4 ; 0 ; 0] ;
x0 = [qi ; dqi ; alpha] ;
% Cost=10 ; Geq=[] ; Gineq=[] ;

%% Main Optimization Loop
options=optimset('fmincon');
opts_fmincon = optimset(options,'MaxFunEvals',1e5,'TolCon',1e-5,'TolFun',1e-6,'TolX',1e-8,'Display','iter','DiffMinChange',0.01) ;
A = [] ; Aeq = [] ; b = []; beq = [] ;
q0 = x0(1:dimq) ;
dq0 = x0(dimq+1:2*dimq) ;
alpha0 = x0(2*dimq+1:end) ;
lb=[];ub=[];

% lb=[q0-0.2*pi; dq0-pi/2; alpha0 - 0.2*pi]; %lb(1)=qzero_minus(1);   % Not really sure
% ub=[q0+0.2*pi; dq0+pi/2; alpha0 + 0.2*pi]; %ub(1)=qzero_minus(1);   % Not really sure

[X,FVAL,exitflag,Output] = fmincon('CostFunc',x0,A,b,Aeq,beq,lb,ub,'NonlinConstraints',opts_fmincon) 
