function [psi,theta,phi] = RotToYPR(R)
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
% ===================================================
theta = atan2(-R(3, 1),sqrt(R(1,1)^2+R(2,1)^2));
% ===================================================

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
