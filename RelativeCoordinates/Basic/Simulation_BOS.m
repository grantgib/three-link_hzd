%% Main Simulation_3link 
% by Grant
% Simulation for the simple 3-link walker with straight knees.
% Simulates starting with beginning of step

clear all
close all
addpath(genpath('Animation_Plots'))
addpath(genpath('Control'))
addpath(genpath('Dynamics'))
addpath(genpath('Poincare_Map'))
addpath(genpath('ODE45_Events'))

%% Variables
pStanceFoot = [0 ; 0] ;  % Initial stance foot position
tSpan = 0:0.005:2 ;          % Time span for ode solver

% Initial Conditions
q_i = [-pi/8; pi/8; pi/6];
dq_i = [1.6; -1.6; 0];
x_init = [q_i; dq_i] ;   % Initial state

%% Variables to be collected and saved
Time = []; X = []; Y = []; DY = []; 
Torques = []; Forces = []; pSwingFoot = []; GRF = [];
T_ReturnMap = [] ; X_ReturnMap = [] ; 
StanceFoot = [] ;

%% Options solver options
impactAngle = x_init(1);
Poincare_angle = 0;

options1 = odeset('Events',@(t,x) Event_Psection_3link(t,x,Poincare_angle),'MaxStep',0.01,'Refine',4,...
    'RelTol',1e-6,'AbsTol',1e-8) ;      % Option for Poincare section

options2 = odeset('Events',@(t,x) EventImpact_3link(t,x,impactAngle),'MaxStep',0.01,'Refine',4,...
    'RelTol',1e-6,'AbsTol',1e-8) ;      % Option for Impact event

%% Main Loop
n = 20 ;    % number of steps to take
disp('Main Dynamics Loop...');
for loopNum = 1:n    
    %% Simulate Dynamics
    % Use ode45 solver to compute dynamics until reaching the Poincare
    % section
    [t1,x1,t_ReturnMap,x_ReturnMap] = ode45(@(t,x) StateSpaceModel_3link(t,x,pStanceFoot), tSpan, x_init, options1) ;
    
    % Continue the solver (IC at section) until impact has occured
    [t2,x2,t_minus,x_minus] = ode45(@(t,x) StateSpaceModel_3link(t,x,pStanceFoot), tSpan, x_ReturnMap, options2) ; 
    x_minus = x_minus' ;       % state space values right before impact
    q_minus = x_minus(1:length(x_minus)/2).' ;
    
    % Correct t2 for the time between the Poincare section and impact. This
    % is not a discrete event and the trajectory is continous from impact
    % result to next impact. t_ReturnMap is a the single time value when
    % the Poincare section occurs
    sz = size(t2) ;
    for s = 1:sz
        t2(s) = t2(s) + t_ReturnMap ;
    end
    
    % Collate time and state data for a single trajectory (step)
    x_traject = [x1 ; x2] ;
    t_traject = [t1 ; t2] ;
    
    %% Return Map data
    % Collect Poincare return map values
     T_ReturnMap = [T_ReturnMap ; t_ReturnMap] ;
     X_ReturnMap = [X_ReturnMap ; x_ReturnMap] ;
    
    %% Store time and state data (for each trajectory or step)
    X = [X ; x_traject] ;
    Time = [Time ; t_traject] ;
        
    %% Reconstruct Dynamics
    [xrow,xcol] = size(x_traject) ;
    for iter = 1:xrow
        t_iter = t_traject(iter) ; 
        x_iter = x_traject(iter,:).' ;
        [dx,q,dq,y,dy,u,pSwingFoot_iter,GRF_iter] = StateSpaceModel_3link(t_iter,...
            x_iter,pStanceFoot) ;
        %[De, Ce_dqe, Ge, Be] = LagrangeModelextend_3link(x_iter) ;
        % Collect Output data
        Y = [Y y] ;
        DY = [DY dy] ;
        Torques = [Torques u] ;
        pSwingFoot = [pSwingFoot pSwingFoot_iter] ;  
        GRF = [GRF GRF_iter] ;
        StanceFoot = [StanceFoot pStanceFoot] ;
    end
    
    %% Impact Model to compute x+ values and switch states
    [q_plus,dq_plus] = ImpactModel_3link(x_minus) ;
    x_init = [q_plus ; dq_plus] ;
    pStanceFoot = pSwingFoot_iter ;
    impactAngle = x_init(1);
end

%% Clean up data
disp('Cleaning up data...');
Y = Y.' ;
DY = DY.' ;
Torques = Torques.' ;
pSwingFoot = pSwingFoot.' ;
GRF = GRF.' ;
StanceFoot = StanceFoot.' ;

%% Determine Eigenvalues of the Poincare Map
% disp('Poincare Map...');
% % Find the Fixed Points\
% mapped_state = 1;                   % q1 or th1 = constant is where the section begins
% x_guess = zeros(6,1);               % Should be a 6x1
% x_guess(mapped_state) = x_guess(mapped_state) + angleSwitch; 
% x_FPnr =
% NewtonRaphson_Poincare(x_guess,mapped_state,pStanceFoot,kp,kd,tSpan,options1,options2);   % might return error based on x_guess
% x_FPnr = [angleSwitch; x_FPnr];      % transform into a 6x1 vector 
x_FP = X_ReturnMap(end,:).';         % Assume the fixed point is at the last row of the state return map
% FPerr = x_FPnr - x_FP;
% % Calculate the Jacobian of the Poincare map about the Fixed Point
% perturb = 0.01 ;                    % small pertubation for derivative calculation
% 
% % x_FP should be 6x1
% dP = PoincareJacobian(x_FPnr,mapped_state,perturb,pStanceFoot,kp,kd,tSpan,options1,options2);   % 5x5 Jacobian excluding row and column related to mapped state th1 = constant
% Eig_dP = eig(dP);            % Compute Eigenvalues of Return Map Jacobian 
% Eig_dPabs = abs(Eig_dP);     % Magnitudes of complex eigenvalues

%% Animation and Plots
disp('Animation and plots...');
% Walking animation
figure 
Animation_3link(Time,X,StanceFoot) ;

% Plot state space variables
figure
ss = true ;
PlotOutput_3link(ss,Time,X,Y,DY,Torques,pSwingFoot) ;

% Plot other output data
figure
ss = false ;
PlotOutput_3link(ss,Time,X,Y,DY,Torques,pSwingFoot,GRF) ;

% Plot Poincare Return Map (time-based)
% figure
% iteration = 1:1:n;
% scatter(iteration, X_ReturnMap(:,2),'filled') ;
% hold on ;
% scatter(iteration, X_ReturnMap(:,3),'filled') ;
% hold on 
% scatter(iteration, X_ReturnMap(:,4),'filled') ;
% hold on
% scatter(iteration, X_ReturnMap(:,5),'filled') ;
% hold on
% scatter(iteration, X_ReturnMap(:,6),'filled') ;
% title('Poincare Section: State Values ($\theta_1 = 0^{o}$)','interpreter','latex') ;
% xlabel('$\theta_1 = 0$','interpreter','latex') ;
% ylabel('Return map value','interpreter','latex') ;
% legend({'$\theta_2$','$\theta_3$','$\dot{\theta}_1$','$\dot{\theta}_2$','$\dot{\theta}_3$'},'interpreter','latex') ;

%% Analyzing GRFs
disp('Ground Reaction Forces');
[g,m,m_H,m_T,r,r_T,m_Total] = ModelParam_3link() ;
Fnorm = m_Total * 9.81 ;            % Normal force of robot weight
coeff_fric = 2/3 ;                  % Coefficient of friction
Ffric = coeff_fric*Fnorm ;          % Static Friction force
nSamples = length(Time) ;
F_HorizErr = zeros(nSamples,1) ;
F_VertErr = zeros(nSamples,1) ;

% In order for the robot not to slip, F_HorizErr > 0
for i = 1:nSamples
    F_HorizErr(i) = Ffric - GRF(i,1) ;
end

% In order not to lift up, F_VertErr > 0
for i = 1:nSamples
    F_VertErr(i) = Fnorm - GRF(i,2) ;
end

% figure
% subplot(1,2,1) ;
% sz = 0.5;
% scatter(Time,F_HorizErr,sz) ;
% title('Extra Horizontal Force until Slip','interpreter','latex') ;
% subplot(1,2,2) ;
% scatter(Time,F_VertErr,sz) ;
% title('Extra Vertical Force until Lift','interpreter','latex') ;

%% Exiting 
disp(X(end,:)');
disp('*Done*');