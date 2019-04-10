
%% Parameters
% b = sqrt(gain);
% TL = [-sqrt(2/b) -1/b; -1 -sqrt(2/b)];
% TR = [sqrt(2/b) -1/b; -1 sqrt(2/b)];
% 
% %% Calculations
% % Something is *-1 and explodes this method...
% s = tf('s');
% H_L1 = gain/(1 - gain*TL(1, 2));
% H_R1 = gain/(1 - gain*TR(1, 2));
% 
% KL = [H_L1*TL(1,1) -H_L1 ; TL(2,1)+TL(2,2)*H_L1*TL(1,1) -TL(2,2)*H_L1];
% KR = [H_R1*TR(1,1) -H_R1 ; TR(2,1)+TR(2,2)*H_R1*TR(1,1) -TR(2,2)*H_R1];

%% Matrix
TL = [-inv(B)*sqrt(2) -inv(B)^2; -eye(2) -inv(B)*sqrt(2)];
TR = [inv(B)*sqrt(2) -inv(B)^2; -eye(2) inv(B)*sqrt(2)];

%% Calculations
% Something is *-1 and explodes this method...
s = tf('s');
H_L1 = inv(eye(2)- Kd*TL(1:2, 3:4))*Kd;
H_R1 = inv(eye(2)- Kd*TR(1:2, 3:4))*Kd;

KL = [H_L1*TL(1:2,1:2) -H_L1 ; TL(3:4,1:2)+TL(3:4,3:4)*H_L1*TL(1:2,1:2) -TL(3:4,3:4)*H_L1];
KR = [H_R1*TR(1:2,1:2) -H_R1 ; TR(3:4,1:2)+TR(3:4,3:4)*H_R1*TR(1:2,1:2) -TR(3:4,3:4)*H_R1];
