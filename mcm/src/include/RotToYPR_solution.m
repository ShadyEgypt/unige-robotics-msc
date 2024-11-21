function [psi,theta,phi] = rotToYPR(R)
% Given a rotation matrix the function outputs the relative euler angles
% usign the convention YPR
% Input:
% R rotation matrix
% Output:
% psi angle around z axis (yaw)
% theta angle around y axis (theta)
% phi angle around x axis (phi)
% SUGGESTED FUNCTIONS
    % atan2()
    % sqrt()

% Ensure R is a valid 3x3 matrix
if ~isequal(size(R), [3, 3])
    error('Input matrix R is not a 3x3 matrix.');
end

% Calculate theta (pitch)
theta = -asin(R(3, 1));

% Check for the singularity case where cos(theta) = 0
if abs(cos(theta)) > 1e-6  % cos(theta) is non-zero
    % Calculate psi (yaw) and phi (roll)
    psi = atan2(R(2, 1), R(1, 1));  % psi (yaw)
    phi = atan2(R(3, 2), R(3, 3));  % phi (roll)
else
    % In the singular case, we can define psi or phi to be zero
    psi = atan2(-R(1, 2), R(2, 2));
    phi = 0;
end
psi = psi * (180 / pi);
theta = theta * (180 / pi);
phi = phi * (180 / pi);
end

R1 = [1 0 0; 0 0 -1; 0 1 0];
R2 = [1/2 -sqrt(3)/2 0; sqrt(3)/2 1/2 0; 0 0 1];
R3 = [0 -sqrt(2)/2 sqrt(2)/2; 0.5 sqrt(2)*sqrt(3)/4 -sqrt(2)/4; -sqrt(3)/2 sqrt(2)/4 sqrt(2)/4];

% Store the matrices in a cell array for easy iteration
rotation_matrices = {R1, R2, R3};

% Iterate over each matrix and compute YPR angles
for i = 1:length(rotation_matrices)
    R = rotation_matrices{i};
    [psi, theta, phi] = rotToYPR(R);
    fprintf('For R%d:\n', i);
    fprintf('Psi (Yaw) = %.4f, Theta (Pitch) = %.4f, Phi (Roll) = %.4f\n\n', psi, theta, phi);
end