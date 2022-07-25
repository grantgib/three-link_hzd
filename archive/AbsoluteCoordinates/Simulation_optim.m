%% Main Simulation_3link 
% Summer 2018 - Grant
% Simulation for the simple 3-link walker with straight knees.
clear
clear global
% close all
addpath(genpath('Bezier_Files'))
addpath(genpath('Controller'))
addpath(genpath('Dynamics_Geometry'))
addpath(genpath('Impact_Poincare'))
addpath(genpath('Gait_Design'))
addpath(genpath('Plots'))
%% Global Variables
GlobalVar

%% Initial conditions
% Positions
th1_i = -pi/8 ; th2_i = pi/8 ; th3_i = pi/6 ;        
thi = [th1_i ; th2_i ; th3_i] ;
% Velocities
dth1_i = 1.6 ; dth2_i = -1.6 ; dth3_i = 0 ;         
dthi = [dth1_i ; dth2_i ; dth3_i] ;
x_init = [thi ; dthi] ;

% Coordinate transformation
% qi = [1 0 -1; 0 1 -1; 0 0 1]*thi - [pi ; pi ; 0] ;
qi = [7*pi/8 - pi/6 ; 9*pi/8 - pi/6 ; pi/6] ;
dqi = [1 0 -1; 0 1 -1; 0 0 1]*dthi ;

x_init = [qi ; dqi] ;  
ic = x_init ;

pStanceFoot = [0 ; 0] ; 
tSpan = [0:0.005:2] ;           

%% Variables to be collected and saved
Time = []; X = []; H = []; DH = []; 
Torques = []; Forces = []; pSwingFoot = []; GRF = [];
T_ReturnMap = [] ; X_ReturnMap = [] ; 
StanceFoot = [] ; S = [] ;

%% Proportional-derivative coefficients
% Ts = 0.6 ;              % desired settling time [sec]
% dRatio = 0.8 ;          % desired damping ratio
% wn = 3.9/(dRatio*Ts) ;  % undamped frequencty. wn ~= 3.9/(dRatio*Ts)
% kp = wn^2 ;             % proportional constant ~=66
% kd = 2*dRatio*wn ;      % derivative constant ~=13
kp = 64 ; kd = 16 ;
%% Options solver options
options2 = odeset('Events',@Event_EOS,'MaxStep',0.01,'Refine',4,...
    'RelTol',1e-6,'AbsTol',1e-8) ;

%% Main Loop
n = 1 ;    % number of steps to take
for loopNum = 1:n    
    %% Simulate Dynamics
    % Use ode45 solver to compute dynamics until end of step
    [t1,x1,t_minus,x_minus] = ode45(@(t,x) StateSpaceModel_optim...
        (x), tSpan, x_init, options2) ;
    % Store state and time data
    X = [X ; x1] ;
    Time = [Time ; t1] ;
    %% Reconstruct Dynamics
    [xrow,xcol] = size(x1) ;
    IntegralSqTorque = 0 ;
    for iter = 1:xrow
        t_iter = t1(iter) ; 
        x_iter = x1(iter,:).' ;
        [dx,q,dq,h,dh,u,pSwingFoot_iter,GRF_iter,s] = ...
            StateSpaceModel_optim(x_iter) ;
        % Collect Output data
        H = [H h] ;
        DH = [DH dh] ;
        Torques = [Torques u] ;
        pSwingFoot = [pSwingFoot pSwingFoot_iter] ;  
        GRF = [GRF GRF_iter] ;
        StanceFoot = [StanceFoot pStanceFoot] ;
        S = [S s] ;
    end
    
    %% Impact Model to compute x+ values and switch states
    x_minus = x_minus.' ;
    [q_plus,dq_plus] = ImpactModel_optim(x_minus) ;
    x_init = [q_plus ; dq_plus] ;
    pStanceFoot = pSwingFoot_iter ;
    
end

%% Clean up data
H = H.' ;
DH = DH.' ;
Torques = Torques.' ;
pSwingFoot = pSwingFoot.' ;
GRF = GRF.' ;
StanceFoot = StanceFoot.' ;
S = S.' ;

%% Animation and Plots
% Walking animation
% figure
% Animation_3link(X,StanceFoot) ;

% figure
% subplot(2,3,1) ;
%     plot(Time,Y(:,1)) ;
%     title('y1-torso') ;
%     subplot (2,3,2) ;
%     plot(Time, Torques(:,1)) ;
%     title('u1') ;
%     
%     subplot (2,3,4) ;
%     plot(Time, Y(:,2)) ;
%     title('y2-legs') ;
%     subplot (2,3,5) ;
%     plot(Time, Torques(:,2)) ;
%     title('u2') ;
    
figure(1)
subplot(1,2,1);plot(S,H(:,1)); title('h_1'); xlabel('s') ; 
subplot(1,2,2);plot(S,H(:,2)); title('h_2'); xlabel('s') ; 

figure(2)
subplot(1,2,1);plot(Time,H(:,1)); title('h_1'); xlabel('t') ; 
subplot(1,2,2);plot(Time,H(:,2)); title('h_2'); xlabel('t') ;


%% Analyzing GRFs
% [g,m,m_H,m_T,r,r_T,m_Total] = ModelParam_3link() ;
% Fnorm = m_Total * 9.81 ;            % Normal force of robot weight
% coeff_fric = 2/3 ;                  % Coefficient of friction
% Ffric = coeff_fric*Fnorm ;          % Static Friction force
% nSamples = length(Time) ;
% F_HorizErr = zeros(nSamples,1) ;
% F_VertErr = zeros(nSamples,1) ;
% 
% % In order for the robot not to slip, F_HorizErr > 0
% for i = 1:nSamples
%     F_HorizErr(i) = Ffric - GRF(i,1) ;
% end
% 
% % In order not to lift up, F_VertErr > 0
% for i = 1:nSamples
%     F_VertErr(i) = Fnorm - GRF(i,2) ;
% end
% 
% figure
% subplot(1,2,1) ;
% plot(Time,F_HorizErr) ;
% title('Extra Horizontal Force until Slip') ;
% subplot(1,2,2) ;
% plot(Time,F_VertErr) ;
% title('Extra Vertical Force until Lift') ;
