clear
clf
s = linspace(0,1);
% slope at s=0 -> M(a1 - a0) = 0.5
% slope at s=1 -> M(a5 - a4) = 0.5

%% alphas
alphas = [0;0;0;0] ;
a2_h1 = alphas(1) ;
a3_h1 = alphas(2) ;
a2_h2 = alphas(3) ;
a3_h2 = alphas(4) ;

%% h1
nominalTorsoAngle = pi/6;
slope_begin = 2 ;
slope_end = slope_begin;
M = 5;
a0_h1 = 0;
a5_h1 = a0_h1;
a1_h1 = a0_h1 + (slope_begin/M);
a4_h1 = a5_h1 - (slope_end/M);
% hd_h1 = bezier([a0_h1 a1_h1 a2_h1 a3_h1 a4_h1 a5_h1],s) + nominalTorsoAngle;

for i = 1:100
    bi(i) = bezier([a0_h1 a1_h1 a2_h1 a3_h1 a4_h1 a5_h1],s(i)) + nominalTorsoAngle ;
end
figure(1)
plot(s,bi)
title('h1 - torso')
%% h2

slope_begin = -0.1 ;
slope_end = slope_begin;
M = 5;
a0_h2 = 0;
a5_h2 = a0_h2;
a1_h2 = a0_h2 + (slope_begin/M);
a4_h2 = a5_h2 - (slope_end/M);
% hd_h2 = bezier([a0_h2 a1_h2 a2_h2 a3_h2 a4_h2 a5_h2],s);

for i = 1:100
    bi(i) = bezier([a0_h2 a1_h2 a2_h2 a3_h2 a4_h2 a5_h2],s(i)) ;
end
figure(2)
plot(s,bi)
title('h2 - mirror')