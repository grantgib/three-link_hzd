function [] = PlotOutput_3link(ss,Time,X,Y,DY,Torques,pSwingFoot, GRF) 
% PlotOutput_3link - plot various outputs computed from the simulation

if ss   % do you want to plot the state space variables
    subplot(2,3,1) ;
    plot(Time,X(:,1)) ;
    title('\theta_1') ;

  	subplot(2,3,2) ;
    plot(Time,X(:,2)) ;
    title('\theta_2') ;
    
    subplot(2,3,3) ;
    plot(Time,X(:,3)) ;
    title('\theta_3') ;
    
    subplot(2,3,4) ;
    plot(Time,X(:,4)) ;
    title('d \theta_1') ;
    
    subplot(2,3,5) ;
    plot(Time,X(:,5)) ;
    title('d \theta_2') ;
    
    subplot(2,3,6) ;
    plot(Time,X(:,6)) ;
    title('d \theta_3') ;
       
else   % do you want to plot all other outputs
    subplot(2,3,1) ;
    plot(Time,Y) ;
    legend('y1-torso','y2-legs') ;
    title('Y') ;

    subplot (2,3,2) ;
    plot(Time, DY) ;
    legend('dy1','dy2') ;
    title('DY') ;

    subplot (2,3,3) ;
    plot(Time, Torques) ;
    legend('u1','u2') ;
    title('Torques') ;

    subplot (2,3,4) ;
    plot(Time, pSwingFoot(:,1)) ;
    title('SwingFoot x') ;
    
    subplot (2,3,5) ;
    plot(Time, GRF(:,1)) ;
    title('GRF horizontal') ;
    
    subplot (2,3,6) ;
    plot(Time, GRF(:,2)) ;
    title('GRF vertical') ;
    
end

