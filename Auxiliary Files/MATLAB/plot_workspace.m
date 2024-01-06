% 2-DOF Revolute Robotic Arm Workspace Calculation

% Define the joint angles
theta1 = linspace(0, 100/180*pi, 600); % Joint angle 1 varies from 0 to 2*pi
theta2 = linspace(0, pi/2, 100); % Joint angle 2 varies from 0 to 2*pi

% Define the link lengths
L1 = 81; % Length of the first link
L2 = 108.5; % Length of the second link

% Initialize variables to store end-effector positions
x = zeros(length(theta1), length(theta2));
y = zeros(length(theta1), length(theta2));

% Calculate end-effector positions for all combinations of joint angles
for i = 1:length(theta1)
    for j = 1:length(theta2)
        % Forward kinematics to compute end-effector position
        x(i, j) = L1 * cos(theta1(i)) + L2 * cos(theta1(i) + theta2(j));
        y(i, j) = L1 * sin(theta1(i)) + L2 * sin(theta1(i) + theta2(j));
    end
end

% Plot the workspace
figure;
plot(x(:), y(:), 'b.'); % Scatter plot of end-effector positions
title('2-DOF Revolute Robotic Arm Workspace');
xlabel('Abscissa distance (mm)');
ylabel('Ordinate distance (mm)');
axis equal; % Set equal scaling for x and y axes
grid on;
