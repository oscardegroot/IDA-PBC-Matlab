%% Calculates the network gain of the scattering transformation

% Scattering Transformation Matrices
TL = [-inv(B)*sqrt(2) -inv(B)^2; -eye(2) -inv(B)*sqrt(2)];
TR = [inv(B)*sqrt(2) -inv(B)^2; -eye(2) inv(B)*sqrt(2)];

%WVML = [1/(sqrt(2)) -0];

% Calculate the transfer function (which is a gain)
%s = tf('s');
H_L1 = inv(eye(2)- Kd*TL(1:2, 3:4))*Kd;
H_R1 = inv(eye(2)- Kd*TR(1:2, 3:4))*Kd;

KL = [H_L1*TL(1:2,1:2) -H_L1 ; TL(3:4,1:2)+TL(3:4,3:4)*H_L1*TL(1:2,1:2) -TL(3:4,3:4)*H_L1];
KR = [H_R1*TR(1:2,1:2) -H_R1 ; TR(3:4,1:2)+TR(3:4,3:4)*H_R1*TR(1:2,1:2) -TR(3:4,3:4)*H_R1];

clear TL TR s H_L1 H_R1