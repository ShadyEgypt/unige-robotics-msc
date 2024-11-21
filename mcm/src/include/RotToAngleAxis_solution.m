function [h,theta] = RotToAngleAxis(R)
%EULER REPRESENTATION: Given a tensor rotation matrices this function
% should output the equivalent angle-axis representation values,
% respectively 'theta' (angle), 'h' (axis) 
% SUGGESTED FUNCTIONS
    % size()
    % eye()
    % abs()
    % det()
    % NB: Enter a square, 3x3 proper-orthogonal matrix to calculate its angle
    % and axis of rotation. Error messages must be displayed if the matrix
    % does not satisfy the rotation matrix criteria.

    % Check if R is a 3x3 matrix
    if ~isequal(size(R), [3, 3])
        error('Input matrix R is not a 3x3 matrix.');
    end
    
    % Check if R has a determinant close to 1
    if abs(det(R) - 1) > 1e-6
        error('Input matrix R does not have a determinant of 1.');
    end
    
    % Check if R is orthogonal
    if max(max(abs(R * R' - eye(3)))) > 1e-6
        error('Input matrix R is not orthogonal (R * R'' != I).');
    end
    
    % Calculate the rotation angle theta
    theta = acos((trace(R) - 1) / 2);

    % Handle the case when theta is zero (no rotation)
    if abs(theta) < 1e-6
        h = [1; 0; 0]; % Arbitrary axis
        theta = 0;
        return;
    end
    
    % Calculate the rotation axis h
    if abs(theta - pi) < 1e-6
        % Special case when theta is pi (180-degree rotation)
        h = sqrt((diag(R) + 1) / 2);
        h = h .* (sign(vex(R)) .* (h > 0));
    else
        % General case
        h = 1 / (2 * sin(theta)) * vex(R - R');
    end
    
    % Convert theta from radians to degrees
    theta = theta * (180 / pi);
end

function [h1, h2, theta] = RotToAngleAxis2(R)
    % EULER REPRESENTATION: Given a tensor rotation matrix, this function
    % outputs the equivalent angle-axis representation values,
    % namely 'theta' (angle) and two possible axes 'h1' and 'h2'.
    % NB: Enter a square, 3x3 proper-orthogonal matrix to calculate its angle
    % and axis of rotation. Error messages must be displayed if the matrix
    % does not satisfy the rotation matrix criteria.

    % Check if R is a 3x3 matrix
    if ~isequal(size(R), [3, 3])
        error('Input matrix R is not a 3x3 matrix.');
    end
    
    % Check if R is orthogonal
    if max(max(abs(R * R' - eye(3)))) > 1e-6
        error('Input matrix R is not orthogonal (R * R'' != I).');
    end
    
    % Calculate the rotation angle theta
    theta = acos((trace(R) - 1) / 2);

    % Handle the case when theta is zero (no rotation)
    if abs(theta) < 1e-6
        h1 = [1; 0; 0]; % Arbitrary axis
        h2 = [1; 0; 0]; % Arbitrary axis, same as h1 in this case
        theta = 0;
        return;
    end
    
    % Calculate the rotation axis h for the case theta = pi
    if abs(theta - pi) < 1e-6
        % Special case when theta is pi (180-degree rotation)
        h1 = zeros(3, 1);
        h2 = zeros(3, 1);
        
        % Compute |h_i| for each component
        for i = 1:3
            h1(i) = sqrt((R(i, i) + 1) / 2);
        end

        % Ensure at least one component in h1 has an absolute value > 0
        % Starting from any component with |h_i| > 0
        % We check each component and find the first non-zero to generate h2
        
        if abs(h1(1)) > 1e-6
            h2 = [-h1(1); h1(2); h1(3)]; % Second solution by negating h1(1)
        elseif abs(h1(2)) > 1e-6
            h2 = [h1(1); -h1(2); h1(3)]; % Second solution by negating h1(2)
        elseif abs(h1(3)) > 1e-6
            h2 = [h1(1); h1(2); -h1(3)]; % Second solution by negating h1(3)
        else
            error('Unexpected error: No component in h1 has an absolute value > 0.');
        end

        % Adjust the signs of each solution based on the off-diagonal elements
        h1(1) = h1(1) * sign(R(2, 3)); % Sign based on R23
        h1(2) = h1(2) * sign(R(1, 3)); % Sign based on R13
        h1(3) = h1(3) * sign(R(1, 2)); % Sign based on R12

        h2(1) = h2(1) * sign(R(2, 3)); % Sign based on R23
        h2(2) = h2(2) * sign(R(1, 3)); % Sign based on R13
        h2(3) = h2(3) * sign(R(1, 2)); % Sign based on R12

    else
        % General case
        h1 = 1 / (2 * sin(theta)) * vex(R - R');
        h2 = h1; % Only one solution in the general case
    end
    
    % Convert theta from radians to degrees
    theta = theta * (180 / pi);
end
 
function a = vex(S_a)
    % input: skew matrix S_a (3x3)
    % output: the original a vector (3x1)
    % Check if S_a is a 3x3 skew-symmetric matrix
    if ~isequal(size(S_a), [3, 3]) || ~isequal(S_a, -S_a')
        error('Input matrix S_a must be a 3x3 skew-symmetric matrix.');
    end
    
    % Extract the vector a from the skew-symmetric matrix S_a
    a = [S_a(3, 2); S_a(1, 3); S_a(2, 1)];
end

% Define the rotation matrices from the problem statement
R1 = [1 0 0; 0 0 -1; 0 1 0];          % Q2.2
R2 = [0.5 -sqrt(3)/2 0; sqrt(3)/2 0.5 0; 0 0 1];  % Q2.3
R3 = [1 0 0; 0 1 0; 0 0 1];            % Q2.4
R4 = [-1 0 0; 0 1 0; 0 0 1];           % Q2.5

% Apply the function to each matrix and display results
matrices = {R1, R2, R3, R4};
for i = 1:length(matrices)
    try
        [h, theta] = RotToAngleAxis2(matrices{i});
        fprintf('Matrix Q2.%d:\n', i+1);
        fprintf('Axis h: [%.4f, %.4f, %.4f]\n', h);
        fprintf('Angle theta: %.4f degrees\n\n', theta);
    catch ME
        fprintf('Matrix Q2.%d:\n', i+1);
        fprintf('Error: %s\n\n', ME.message);
    end
end