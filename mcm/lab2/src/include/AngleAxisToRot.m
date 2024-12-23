function R = AngleAxisToRot(h, theta)
    % The function implements Rodrigues' Rotation Formula
    % Input: 
    % h is the axis of rotation (a unit vector)
    % theta is the angle of rotation in radians
    % Output:
    % R is the corresponding rotation matrix

    % Ensure that h is a unit vector (normalize if necessary)
    h = h / norm(h);

    % Skew-symmetric matrix for the axis of rotation
    x = [ 0    -h(3)   h(2);
          h(3)   0    -h(1);
         -h(2)   h(1)   0 ];

    % Rodrigues' rotation formula
    R = eye(3) + sin(theta) * x + (1 - cos(theta)) * (x * x);
end
