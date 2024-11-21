function [R] = YPRToRot(psi, theta, phi)
% The function compute the rotation matrix using the YPR (yaw-pitch-roll)
% convention, given psi, theta, phi.
% Input:
% psi angle around z axis (yaw)
% theta angle around y axis (theta)
% phi angle around x axis (phi)
% Output:
% R rotation matrix
% Calculate individual rotation matrices for each angle
Rz = [cos(psi), -sin(psi), 0;
      sin(psi), cos(psi), 0;
      0, 0, 1];
  
Ry = [cos(theta), 0, sin(theta);
      0, 1, 0;
      -sin(theta), 0, cos(theta)];
  
Rx = [1, 0, 0;
      0, cos(phi), -sin(phi);
      0, sin(phi), cos(phi)];
  
% Calculate the combined rotation matrix using Z-Y-X order
R = Rz * Ry * Rx;
end

% Define test cases
test_cases = [
    struct('psi', 0, 'theta', 0, 'phi', pi/2), % Case Q3.2
    struct('psi', deg2rad(60), 'theta', 0, 'phi', 0), % Case Q3.3
    struct('psi', pi/3, 'theta', pi/2, 'phi', pi/4), % Case Q3.4
    struct('psi', 0, 'theta', pi/2, 'phi', -pi/12) % Case Q3.5
];

% Loop through each test case and calculate rotation matrix
for i = 1:length(test_cases)
    tc = test_cases(i);
    R = YPRToRot(tc.psi, tc.theta, tc.phi);
    
    % Display the result for the current test case
    fprintf('Test Case Q3.%d\n', i + 1);
    disp('Rotation Matrix:');
    disp(R);
    fprintf('\n');
end