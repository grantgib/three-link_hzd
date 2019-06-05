% GAIT_DESIGN_FMINCON - Use fmincon to design optimized gaits that are
% defined by the bezier polynomials
clear all

addpath(genpath('../../GrizzleCoordinateSystem'));
% addpath('../Animation_Plots');
% addpath('../Control');
% addpath('../Control/Bezier_Files');
% addpath('../Dynamics');
% addpath('../ODE45_Events');
% addpath('../Poincare_Map');

%% INITIALIZE VARIABLES
disp('Initialize Variables...');
% Global Variables
myGlobalVariables

% General Variables [Starting with end of step (x_minus)]
counter = 0;
x_ic = [pi/8 pi/8 1.6 -1.5 0.1]';    % q2 not included and is set = -q1 in cost
freeAlphas = [pi/6 pi/6 0 0]'; % alpha 2,3 for y1,y2 are free can be changed by optimization
X0 = [x_ic; freeAlphas];

% Cost = 305.174. Computed from initial guess
X0 = [0.3038
    0.2383
    1.1260
   -0.6731
   -0.0268
    0.1652
    0.1525
    0.6283
   -0.0132]

% Cost = 308.901. Satisfies torque and velocity constraints
X0=[0.3037
    0.2379
    1.1280
   -0.6744
   -0.0338
    0.1676
    0.1500
    0.6286
   -0.0284]

% Cost = 1207. speed is 1.0 w/ no torque constraints
X0=[0.3372
    0.4147
    1.5559
   -0.7245
   -0.1709
   -0.1061
    0.3697
    0.4163
   -0.1033]

% Cost = 496.3686. speed is 1.0 w/ no torque constraints
X0=[0.3005
    0.3725
    1.4807
   -0.9349
   -0.3351
    0.1711
    0.2916
    0.7687
    0.2045]

% Cost = 428.8277. speed=1.0, torque < 37
X0=[0.2957
    0.3845
    1.6560
   -0.0450
   -1.0724
    0.2582
    0.3588
    0.6568
    0.0462]

% Cost = 464.8080. speed=1.0, torque < 40, steplength=0.58
% X0=[0.2942
%     0.3849
%     1.7779
%    -0.6733
%    -1.5367
%     0.0767
%     0.3726
%     0.7158
%     0.1212]
% 
% X0=[0.2107
%     0.8429
%     2.7204
%    -0.4154
%    -0.7849
%     0.6756
%     0.5645
%     0.4991
%     0.5923]
% 
% X0=[0.1432
%     0.8210
%     3.0331
%    -0.4530
%    -0.4707
%     0.6772
%     0.6525
%     0.4295
%     0.3172]

%% FMINCON
disp('Optimization...');
options = optimset('fmincon');
options_fmincon = optimset(options,'MaxFunEvals',1e5,'TolCon',1e-2,'TolFun',1e-2,'TolX',1e-3,'Display','iter','DiffMinChange',0.001) ;
% options_fmincon = optimset(options,'MaxFunEvals',1e5,'TolCon',1e-3,'TolFun',1e-3,'TolX',1e-4,'Display','iter','DiffMinChange',0.001) ;
A=[]; Aeq=[]; b=[]; beq=[];
% bounds used for initial guess
% lb = [0.15; 0; 0.5; -2.0; -0.1; X0(6:7) - 0.2*pi; X0(8:end) - 0.2*pi;]
% ub = [pi/4; pi/4; 2.0; 0; 0.5;  X0(6:7) + 0.2*pi; X0(8:end) + 0.2*pi;]
lb = X0 - 0.15*pi;
ub = X0 + 0.15*pi;

% save('OptimGaitOld','X0');
X0

[X,FVAL,exitflag,Output] = fmincon('Cost_Torque',X0,A,Aeq,b,beq,lb,ub,'Nonlinear_Constraints',options_fmincon)

% X_SAVE = [X]; 
% save('OptimGait','X_SAVE');

%% CONSTRAINT CHECKER
% Points_3link(X_SAVE(1:6),pStanceFoot);

%% EXITING
disp('Done'); 










