clear
close all
s = linspace(0,1);
% slope at s=0 -> M(a1 - a0) = 0.5
% slope at s=1 -> M(a5 - a4) = 0.5

%% alphas
alphas = [-0.5;-0.5;0;0] ;
a2_1 = alphas(1) ;
a3_1 = alphas(2) ;
a2_2 = alphas(3) ;
a3_2 = alphas(4) ;

%% h1
slope = .1 ; M = 5 ;
a0 = 0 ;a1 = (slope/M)+a0 ;a4 = -(slope/M) ; a5 = a0 ;
a2 = a2_1 ; a3 = a3_1 ;

for i = 1:100
    bi(i) = bezier([a0 a1 a2 a3 a4 a5],s(i)) ;
end
figure
plot(s,bi-pi/6)
title('h1 - torso')
%% h2

slope = .1 ; M = 5 ;
a0 = 0 ;                a5 = a0 ;
a1 = (slope/M)+a0 ;     a4 = a5 - (slope/M) ; 
a2 = a2_2 ;             a3 = a3_2 ;

for i = 1:100
    bi(i) = bezier([a0 a1 a2 a3 a4 a5],s(i)) ;
end
figure
plot(s,bi)
title('h2 - mirror')