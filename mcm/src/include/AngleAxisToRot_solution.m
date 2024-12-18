function R = AngleAxisToRot(h,theta)
% The fuction implement the Rodrigues Formula
% Input: 
% h is the axis of rotation
% theta is the angle of rotation (rad)
% Output:
% R rotation matrix

x = [ 0    -h(3)  h(2);
      h(3)  0    -h(1);
     -h(2)  h(1)  0];

mp = x;
R = eye(3) + sin(theta) * mp + (1 - cos(theta)) * (mp^2);
end


% Testing the AngleAxisToRot function

% Case Q1.2: h = [1, 0, 0]T and θ = 90 degrees (π/2 radians)
h1 = [1, 0, 0];
theta1 = pi / 2; % Convert 90 degrees to radians
R1 = AngleAxisToRot(h1, theta1);
disp('Rotation matrix for Q1.2 (h = [1, 0, 0], θ = 90°):');
disp(R1);

% Case Q1.3: h = [0, 0, 1]T and θ = π/3
h2 = [0, 0, 1];
theta2 = pi / 3; % Already in radians
R2 = AngleAxisToRot(h2, theta2);
disp('Rotation matrix for Q1.3 (h = [0, 0, 1], θ = π/3):');
disp(R2);

% Case Q1.4: ρ = [-π/3, -π/6, π/3]; ρ = hθ
rho = [-pi/3, -pi/6, pi/3];
theta3 = norm(rho); % θ is the magnitude of ρ
h3 = rho / theta3; % h is the unit vector in the direction of ρ
R3 = AngleAxisToRot(h3, theta3);
disp('Rotation matrix for Q1.4 (ρ = [-π/3, -π/6, π/3]):');
disp(R3);
