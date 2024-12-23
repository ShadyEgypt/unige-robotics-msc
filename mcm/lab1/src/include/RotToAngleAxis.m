function [h, theta] = RotToAngleAxis(R)
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

    % check if the det is 1
    if abs(det(R) - 1) > 1e-6
        error('The determinant of the rotation matrix is NOT equal to 1.');
    end
    % Calculate the rotation angle theta
    h_list = [];
    theta = acos((trace(R) - 1) / 2);
    % Handle the case when theta is zero (no rotation)
    if abs(theta) < 1e-6
        h = [1; 0; 0]; % Arbitrary axis
        theta = 0;
        return;
    % Handle the case when theta is pi
    elseif abs(theta - pi) < 1e-10
        h1_1 = sqrt((R(1,1) + 1) / 2);
        h1_2 = sqrt((R(2,2) + 1) / 2);
        h1_3 = sqrt((R(3,3) + 1) / 2);

        h1 = [abs(h1_1), abs(h1_2), abs(h1_3)];
        [first_h, idx] = max(h1);

        % Set the sgn of ather components with repect R's compoents
        if idx == 1
            second_h = sign(first_h)*sign(R(1,2)*sqrt((R(1,2) + 1) / 2));
            third_h = sign(first_h)*sign(R(1,3)*sqrt((R(1,3) + 1) / 2));
            h = [first_h; second_h; third_h];
            h_list = [h_list, h, -1*h];
        elseif idx == 2
            second_h = sign(first_h)*sign(R(2,1)*sqrt((R(2,1) + 1) / 2));
            third_h = sign(first_h)*sign(R(2,3)*sqrt((R(2,3) + 1) / 2));
            h = [second_h; first_h; third_h];
            h_list = [h_list, h, -1*h];
        elseif idx == 3
            second_h = sign(first_h)*sign(R(3,1)*sqrt((R(3,1) + 1) / 2));
            third_h = sign(first_h)*sign(R(3,2)*sqrt((R(3,2) + 1) / 2));
            h = [second_h; third_h; first_h];
            h_list = [h_list, h, -1*h];
        end
        disp('h list');
        disp(h_list);
    else
        a = vex((R - R') / 2);
        % Axial vector a, a = sin(theta) * h => h = a / sin(theta)
        h = a / sin(theta);
    end

    % Convert theta from radians to degrees
    % theta = theta * (180 / pi);
    % fprintf('theta: %d\n', theta);
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

