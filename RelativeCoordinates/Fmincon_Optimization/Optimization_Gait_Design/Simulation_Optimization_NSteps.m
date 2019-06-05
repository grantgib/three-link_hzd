% SIMULATION_3LINK_OPTIMIZATION - animated simulation showing one step of
% the optimized gait to minimize energy from joint torques

clear all
close all
%% Setup file paths
addpath(genpath('../../GrizzleCoordinateSystem'));
% addpath('../Animation_Plots');
% addpath('../Control');
% addpath('../Control/Bezier_Files');
% addpath('../Dynamics');
% addpath('../ODE45_Events');
% addpath('../Poincare_Map');

%% INITIALIZE VARIABLES
% Global Variables
myGlobalVariables % Cost Geq Gineq alphas X0 angleSwitch=pi/16 counter

% Starting with end of step (x_minus)
x_ic = [pi/8 pi/10 1.6 -1.4 0]';    % q2 not included and is set = -q1 in cost
freeAlphas = [0 0 0 0]'; % alpha 2,3 for y1,y2 are free can be changed by optimization
X0 = [x_ic; freeAlphas];

% load the optimized gait
% load('OptimGait');
% X0 = X_SAVE;

% Cost = 305.174. no constraints
X0 = [0.3038
    0.2383
    1.1260
   -0.6731
   -0.0268
    0.1652
    0.1525
    0.6283
   -0.0132]

% Cost = 240.0974. satisfies torque <50. Speed > 0.5. Ft/Fn < 0.6.
X0=[0.2980
    0.2466
    1.2839
   -0.1215
   -0.7131
    0.1462
    0.0090
    0.4718
    0.3510]

% Cost = 496.3686. speed is 1.0 w/ no torque constraints
% X0=[0.3005
%     0.3725
%     1.4807
%    -0.9349
%    -0.3351
%     0.1711
%     0.2916
%     0.7687
%     0.2045]

% Cost = 507.1545. Velocity=1.0. Torque constraint at 40 and slippage
% constraint applied
X0=[0.3031
    0.3891
    1.5409
   -0.5259
   -0.5194
    0.1780
    0.3431
    0.7853
    0.1244]

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
X0=[0.2942
    0.3849
    1.7779
   -0.6733
   -1.5367
    0.0767
    0.3726
    0.7158
    0.1212]

% v=2.3291
% X0=[0.2107
%     0.8429
%     2.7204
%    -0.4154
%    -0.7849
%     0.6756
%     0.5645
%     0.4991
%     0.5923]

% v=2.53
% X0=[0.1888
%     0.8871
%     2.8775
%    -0.5725
%    -0.6477
%     0.7459
%     0.5417
%     0.5636
%     0.5373]

% v=2.51
% X0=[0.1715
%     0.7895
%     2.8638
%    -0.5725
%    -0.6800
%     0.6885
%     0.6157
%     0.4470
%     0.4357]
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

% X0=[0.1212
%     0.7593
%     3.0791
%    -0.5737
%    -0.3106
%     0.6924
%     0.6352
%     0.4804
%     0.2562]

%% General Variables
dimq = 3;
q1Switch = X0(1);

% ODE solver parameters
options = odeset('Events',@(t,x) OptimEvent_3Link(t,x,q1Switch),'MaxStep',0.1,'Refine',4,'RelTol',10^-5,'AbsTol',10^-6);
tspan = [0 2];
% timeinterv = 0.005;
% tspan=0:timeinterv:2;

% Variables to be collected and save
Time = []; X = []; Y = []; DY = []; 
Torques = []; Forces = []; pSwing = []; GRF = [];
StanceFoot = [] ; S = [] ; 
FT =[]; FN = [];
Velocity = [];
Slip = [];
FR = [];

tic
%% SIMULATE STEP (INCLUDING IMPACT)
% Extract Parameters at End of Step
[q_minus,dq_minus,h_alpha45] = ExtractAtEOS(X0,q1Switch);
x_minus = [q_minus; dq_minus];
pStanceFoot = [0 0]';
StanceFoot = [StanceFoot pStanceFoot];
X = [X x_minus];
Time = [0 Time];
n = 5;

for i = 1:n
[pHip,pTorso,pSwingFoot_end] = Points_3link(x_minus,pStanceFoot) ;

% Apply Impact
[q_plus,dq_plus] = ImpactModel_3link(x_minus);
x_plus = [q_plus; dq_plus];
pStanceFoot = pSwingFoot_end;

% Set alpha parameters
h_alpha01 = ExtractAtBOS(x_plus,q1Switch);
h_alpha23 = X0(6:length(X0));
h_alpha = ReconstructHalpha(h_alpha01,h_alpha23,h_alpha45);
% BezierPlot(X0);

% Initial Condition
x_init = [q_plus; dq_plus];

% ODE Solver
[t1,x1,t_end,x_end] = ode45(@(t,x) StateSpaceModel_3link(t,x,pStanceFoot,q1Switch,h_alpha),tspan,x_init,options);
[pHip,pTorso,pSwingEnd] = Points_3link(x_end,pStanceFoot) ;
StepLength = pSwingEnd(1) - pStanceFoot(1) ;
X = [X x1'];
% Time = [Time t1'+timeinterv];
Time = [Time t1'];

%% RECONSTRUCT DYNAMICS 
% dt = diff(t1);
IntegralSqTorque = 0;
% pSwing = zeros(length(t1)-1,2);
for k = 1:length(t1)
    xiter = x1(k,:);
    [dx,q,dq,y,dy,u,pSwingiter,GRF_iter,s] = StateSpaceModel_3link(t1(k),xiter',pStanceFoot,q1Switch,h_alpha);
%     pSwing(k,:) = pSwingiter';
%     IntegralSqTorque = IntegralSqTorque + norm(u)^2*dt(k) ;
   
    %% Ground Reaction Forces
    [De,Ce_dqe,Ge,Be] = LagrangeModelextend_3link(xiter);
    E1 = [0 0 0 1 0 ; 0 0 0 0 1] ;
    GRF = GRF_3link(De,Ce_dqe,Ge,Be,E1,u) ;
    Ft = GRF(1);
    Fn = GRF(2);
    slip = abs(Ft/Fn);
    velocity = abs(StepLength/t_end);
    
    %% Froude number (ratio of the centripetal force around the center of motion, the foot, and the weight of the animal walking)
    % Fr = v^2/(g*l)
    [pHip,pTorso,pSwingFoot] = Points_3link(xiter,pStanceFoot);
    Fr = velocity / (9.81*pHip(2));
    
   %% Store Data
   Y = [Y y] ;
   DY = [DY dy] ;
   Torques = [Torques u] ;
   S = [S s] ;
   FT = [FT Ft];
   FN = [FN Fn];
   StanceFoot = [StanceFoot pStanceFoot] ;
   pSwing = [pSwing pSwingiter];
   Velocity = [Velocity velocity];
   Slip = [Slip slip];
   FR = [FR Fr];
   
end
%% Prepare for next loop 
x_minus = x_end';

end
%% CLEAN DATA
X = X.';
Time = Time.';
Y = Y.' ;
DY = DY.' ;
Torques = Torques.' ;
S = S.' ;
FT = FT';
FN = FN';
pSwing = pSwing';
StanceFoot = StanceFoot.' ;
Velocity = Velocity';
Slip=Slip';
FR = FR';

%% COST
Cost = IntegralSqTorque / max(0.1/StepLength);

toc
%% ANIMATION + PLOTS
Animation_3link(t1,X,StanceFoot)
% Plot state space variables
ss = true ;
Plots_3link_Optimization(ss,Time,X,Y,DY,S,Torques,pSwing,GRF) ;

% Plot other output data
ss = false ;
Plots_3link_Optimization(ss,Time,X,Y,DY,S,Torques,pSwing,GRF) ;









