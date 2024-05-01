clc
clear
syms theta


d1 = 0.290; a1 = 0; alpha_1 = -pi/2; theta_1 = 0;
d2 = 0; a2 = 0.270; alpha_2 = 0; theta_2 = -pi/2;
d3 = 0; a3 = 0.07; alpha_3 = -pi/2; theta_3 = 0;
d4 = 0.302; a4 = 0; alpha_4 = pi/2; theta_4 = 0;
d5 = 0; a5 = 0; alpha_5 = -pi/2; theta_5 = 0;
d6 = 0.072; a6 = 0; alpha_6 = 0; theta_6 = 0;

findA(theta_6, d6, a6, alpha_6);

function findA(theta_l, d, a, alpha)
    syms theta
    Rot_theta = [cos(theta + theta_l) -sin(theta + theta_l) 0 0;
                 sin(theta + theta_l) cos(theta + theta_l) 0 0;
                 0 0 1 0;
                 0 0 0 1];
    
    Trans_d = [1 0 0 0;
               0 1 0 0;
               0 0 1 d;
               0 0 0 1];
    
    Trans_a = [1 0 0 a;
               0 1 0 0;
               0 0 1 0;
               0 0 0 1];
    
    Rot_alpha = [1 0 0 0;
                  0 cos(alpha) -sin(alpha) 0;
                  0 sin(alpha) cos(alpha) 0;
                  0 0 0 1];
    
    A = Rot_theta*Trans_d*Trans_a*Rot_alpha
end