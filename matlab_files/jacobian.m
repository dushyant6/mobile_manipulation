clc
clear
syms s2 c2 s5 c5;

A1 = [1 0 0 0;
      0 0 1 0;
      0 -1 0 0.29;
      0 0 0 1];

A2 = [s2 c2 0 0.27*s2;
      -c2 s2 0 -0.27*c2;
      0 0 1 0;
      0 0 0 1];

A3 = [1 0 0 0.07;
      0 0 1 0;
      0 -1 0 0;
      0 0 0 1];

A4 = [1 0 0 0;
      0 0 -1 0;
      0 1 0 0.302;
      0 0 0 1];

A5 = [c5 0 -s5 0;
      s5 0 c5 0;
      0 -1 0 0;
      0 0 0 1];

A6 = [1 0 0 0;
      0 1 0 0;
      0 0 1 0.072;
      0 0 0 1];
%Find transformation matrices
T1 = A1;
T2 = A1*A2;
T3 = A1*A2*A3;
T4 = A1*A2*A3*A4;
T5 = A1*A2*A3*A4*A5;
T6 = A1*A2*A3*A4*A5*A6;

%Find z_i and o_i w.r.t. the base frame
z1 = T1(1:3, 3);
z2 = T2(1:3, 3);
z3 = T3(1:3, 3);
z4 = T4(1:3, 3);
z5 = T5(1:3, 3);
z6 = T6(1:3, 3);

o1 = T1(1:3, 4);
o2 = T2(1:3, 4);
o3 = T3(1:3, 4);
o4 = T4(1:3, 4);
o5 = T5(1:3, 4);
o6 = T6(1:3, 4);

%define base frame o and z
z0 = [0; 0; 1];
o0 = [0; 0; 0];

%Get linear velocity component of jacobian

Jv1 = cross(z0,(o6-o0));
Jv2 = cross(z1,(o6-o1));
Jv3 = cross(z2,(o6-o2));
Jv4 = cross(z3,(o6-o3));
Jv5 = cross(z4,(o6-o4));
Jv6 = cross(z5,(o6-o5));

% Get anuglar velocity component of jacobian
Jw1 = z0;
Jw2 = z1;
Jw3 = z2;
Jw4 = z3;
Jw5 = z4;
Jw6 = z5;

%Get jacobian
J1 = [Jv1;Jw1];
J2 = [Jv2;Jw2];
J3 = [Jv3;Jw3];
J4 = [Jv4;Jw4];
J5 = [Jv5;Jw5];
J6 = [Jv6;Jw6];

J = [J1 J2 J3 J4 J5 J6]
%J = round(J, 2);
manip_sq = J*(transpose(J));

%Find manipulability
manip = sqrt(det(manip_sq))








% Define the range of theta2 and theta5 values
theta2_range = linspace(-pi/2, pi, 100); % Adjust the range as needed
theta5_range = linspace(-pi/2, pi, 100); % Adjust the range as needed

% Initialize a matrix to store the values of m
m_values = zeros(length(theta2_range), length(theta5_range));

% Compute m values for each combination of theta2 and theta5
for i = 1:length(theta2_range)
    for j = 1:length(theta5_range)
        % Compute sin and cos of theta2 and theta5
        s2 = sin(theta2_range(i));
        c2 = cos(theta2_range(i));
        s5 = sin(theta5_range(j));
        c5 = cos(theta5_range(j));
        
        % Compute m using the given equation
        %m = (-1.6308e-04 * (c2^2 + s2^2) * (151*s5*c2^3 + 170*s5*c2^2*s2 + 151*s5*c2*s2^2 + 170*s5*s2^3));
        m = (2.6595e-08*(c2^4 + 2*c2^2*s2^2 + s2^4)*(22801*c2^6*s5^2 + 51340*c2^5*s2*s5^2 + 74502*c2^4*s2^2*s5^2 + 102680*c2^3*s2^3*s5^2 + 80601*c2^2*s2^4*s5^2 + 51340*c2*s2^5*s5^2 + 28900*s2^6*s5^2))^(1/2);
        % Store m value in the matrix
        m_values(i, j) = m;
    end
end

% Plot the values of m
figure;
surf(theta2_range, theta5_range, m_values);
%xlabel('Theta2');
%ylabel('Theta5');
zlabel('m');
title('Plot of manipulability for Range of x and z');



%Find end effector position.
%The end effector should be close to the object pose, we will consider only
%those cases to optimize the angles with high manipulability
A7 = [1 0 0 0;
      0 1 0 0;
      0 0 1 0.15;
      0 0 0 1];

T7 = T6*A7

Base_Frame_Height = 0.47;
Object_Height = 0.64;
EE_Height = Object_Height - Base_Frame_Height;
Object_tolerance = 0.05;
EE = T7(3,4)
EE_X = T7(1,4)
EE_Y = T7(2,4)
% Define the range of theta2 and theta5 values
theta2_range = linspace(-pi/3, pi/3, 120); % Adjust the range as needed
theta5_range = linspace(-pi/3, pi/3, 120); % Adjust the range as needed
theta2_range = linspace(-60, 30, 121); % Adjust the range as needed
theta5_range = linspace(-60, 60, 121); % Adjust the range as needed

% Initialize variables to store the maximum value of m and the corresponding values of theta2 and theta5
max_m = -inf;
max_theta2 = NaN;
max_theta5 = NaN;

% Loop over all combinations of theta2 and theta5
for i = 1:length(theta2_range)
    for j = 1:length(theta5_range)
        % Compute sin and cos of theta2 and theta5
        s2 = sin(theta2_range(i)*pi/180);
        c2 = cos(theta2_range(i)*pi/180);
        s5 = sin(theta5_range(j)*pi/180);
        c5 = cos(theta5_range(j)*pi/180);
        
        % Compute m using the given equation
        %m = -1.6308e-04 * (c2^2 + s2^2) * (151*s5*c2^3 + 170*s5*c2^2*s2 + 151*s5*c2*s2^2 + 170*s5*s2^3);
        m = (2.6595e-08*(c2^4 + 2*c2^2*s2^2 + s2^4)*(22801*c2^6*s5^2 + 51340*c2^5*s2*s5^2 + 74502*c2^4*s2^2*s5^2 + 102680*c2^3*s2^3*s5^2 + 80601*c2^2*s2^4*s5^2 + 51340*c2*s2^5*s5^2 + 28900*s2^6*s5^2))^(1/2);
        % Check if the current value of m is greater than the maximum found so far
        ee_pose = 0.3400*c2 - 0.3020*s2 - 0.2220*c2*s5 - 0.2220*c5*s2 + 0.2900;
        ee_pose = 0.3400*c2 - 0.3020*s2 - 0.4020*c2*s5 - 0.4020*c5*s2 + 0.2900;
        if abs(ee_pose - EE_Height) < 0.01
            if m > max_m
                % Update the maximum value of m and the corresponding values of theta2 and theta5
                max_m = m;
                max_theta2 = theta2_range(i)*pi/180;
                max_theta5 = theta5_range(j)*pi/180;
                %disp("Theta 2, Theta 5 = ");
                %disp(rad2deg(theta2_range(i)));
                %disp(rad2deg(theta5_range(j)));
                %disp(ee_pose);
            end
        end
    end
end

ee_z = 0.3400*cos(max_theta2) - 0.3020*sin(max_theta2) - 0.4020*cos(max_theta2)*sin(max_theta5) - 0.4020*cos(max_theta5)*sin(max_theta2) + 0.2900;
ee_x = 0.3020*cos(max_theta2) + 0.3400*sin(max_theta2) + 0.2220*cos(max_theta2)*cos(max_theta5) - 0.2220*sin(max_theta2)*sin(max_theta5);
baseY = -1.31 - ee_x
% Display the optimized values of theta2 and theta5 and the maximum value of m
disp('Optimized values:');
disp(['Theta2: ', num2str(rad2deg(max_theta2)), ' degrees']);
disp(['Theta5: ', num2str(rad2deg(max_theta5)), ' degrees']);
disp(['Maximum value of m: ', num2str(max_m)]);
disp('ee_pose = ');
disp(ee_z);
disp('ee_pose_x = ');
disp(ee_x);


s2 = sin(-240*pi/180);
c2 = cos(-240*pi/180);
s5 = sin(-60*pi/180);
c5 = cos(-60*pi/180);
        
m = (2.6595e-08*(c2^4 + 2*c2^2*s2^2 + s2^4)*(22801*c2^6*s5^2 + 51340*c2^5*s2*s5^2 + 74502*c2^4*s2^2*s5^2 + 102680*c2^3*s2^3*s5^2 + 80601*c2^2*s2^4*s5^2 + 51340*c2*s2^5*s5^2 + 28900*s2^6*s5^2))^(1/2);
% Check if the current value of m is greater than the maximum found so far
ee_pose = 0.3400*c2 - 0.3020*s2 - 0.2220*c2*s5 - 0.2220*c5*s2 + 0.2900;
disp("ee pose for theta2 = -60 : ");
disp(ee_pose);