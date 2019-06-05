function [] = Plots_3link_Optimization(ss,Time,X,Y,DY,S,Torques,pSwing,GRF) 
% PlotOutput_3link - plot various outputs computed from the simulation
sz = 0.5;
if ss   % do you want to plot the state space variables
    S = [0;S];
    figure
    subplot(2,3,1) ;
%     plot(Time,X(:,1)) ;
%     scatter(Time,X(:,1),sz) ;
    scatter(S,X(:,1),sz) ;
    title('$\theta_1$','interpreter','latex') ;
    xlabel('$S$','interpreter','latex') ;
  	
    subplot(2,3,2) ;
%     plot(Time,X(:,2)) ;
%     scatter(Time,X(:,2),sz) ;
    scatter(S,X(:,2),sz) ;
    title('$\theta_2$','interpreter','latex') ;
    
    subplot(2,3,3) ;
%     plot(Time,X(:,3)) ;
%     scatter(Time,X(:,3),sz) ;
    scatter(S,X(:,3),sz) ;
    title('$\theta_3$','interpreter','latex') ;
    
    subplot(2,3,4) ;
%     plot(Time,X(:,4)) ;
%     scatter(Time,X(:,4),sz) ;
    scatter(S,X(:,4),sz) ;
    title('$\dot{\theta}_1$','interpreter','latex') ;
    
    subplot(2,3,5) ;
%     plot(Time,X(:,5)) ;
%     scatter(Time,X(:,5),sz) ;
    scatter(S,X(:,5),sz) ;
    title('$\dot{\theta}_2$','interpreter','latex') ;
    
    subplot(2,3,6) ;
%     plot(Time,X(:,6)) ;
%     scatter(Time,X(:,6),sz) ;
    scatter(S,X(:,6),sz) ;
    title('$\dot{\theta}_3$','interpreter','latex') ;
       
else   % do you want to plot all other outputs
    figure
    subplot(2,2,1);
    scatter(S,Y(:,1),sz) ;
    ylabel('$y_1: torso$','interpreter','latex');
    xlabel('$S$','interpreter','latex');
    title('$Y_1$','interpreter','latex') ;
    
    subplot(2,2,2);
    scatter(S,Y(:,2),sz) ;
    ylabel('$y_2: mirror$','interpreter','latex');
    xlabel('$S$','interpreter','latex');
    title('$Y_2$','interpreter','latex') ;

    subplot(2,2,3);
    scatter(S,DY(:,1),sz) ;
    ylabel('$\dot{y}_1: torso$','interpreter','latex');
    xlabel('$S$','interpreter','latex');
    title('$\dot{Y}_1$','interpreter','latex') ;
    
    subplot(2,2,4);
    scatter(S,DY(:,2),sz);
    ylabel('$\dot{y}_2: mirror$','interpreter','latex');
    xlabel('$S$','interpreter','latex');
    title('$\dot{Y}_2$','interpreter','latex') ;

    figure
    subplot(2,2,1);
    scatter(S, Torques(:,1),sz) ;
    ylabel('$u_1$','interpreter','latex');
    
    subplot(2,2,2);
    scatter(S, Torques(:,2),sz) ;
    ylabel('$u_2$','interpreter','latex');
    
    subplot(2,2,3);
    scatter(S, pSwing(:,1),sz) ;
    title('SwingFoot x','interpreter','latex') ;
    
%     subplot (2,3,5) ;
% %     plot(Time, GRF(:,1)) ;
%     scatter(S, GRF(:,1),sz) ;
%     title('GRF horizontal','interpreter','latex') ;
%     
%     subplot (2,3,6) ;
% %     plot(Time, GRF(:,2)) ;
%     scatter(S, GRF(:,2),sz) ;
%     title('GRF vertical','interpreter','latex') ;
    
end

