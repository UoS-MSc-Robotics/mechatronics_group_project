% inverse kinematics
% Define the position of the end effector and link lengths
x = 12; % x-coordinate of the end effector
y = 14; % y-coordinate of the end effector
L1 = 8.1; % Length of link 1
L2 = 11; % Length of link 2

% Calculate the joint angles for both solutions
c_theta2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);

% Solution 1
s_theta2_1 = sqrt(1 - c_theta2^2);
theta2_1 = atan2(s_theta2_1, c_theta2);
k1_1 = L1 + L2 * c_theta2;
k2_1 = L2 * s_theta2_1;
theta1_1 = atan2(y, x) - atan2(k2_1, k1_1);
theta1_1 =rad2deg(theta1_1);

% Solution 2
s_theta2_2 = -sqrt(1 - c_theta2^2); % Use the negative root
theta2_2 = atan2(s_theta2_2, c_theta2);
k1_2 = L1 + L2 * c_theta2;
k2_2 = L2 * s_theta2_2;
theta1_2 = atan2(y, x) - atan2(k2_2, k1_2);
theta1_2 = rad2deg(theta1_2);

% Display the joint angles for both solutions
disp([theta1_1, theta2_1]); % Solution 1
disp([theta1_2, theta2_2]); % Solution 2
