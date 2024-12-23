addpath('include');

% TO DO: Test assignment 1 MCM 2024-2025
% 1.1 Angle-axis to rot

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

% 1.2 Rot to angle-axis
% Define test cases
R1 = [1 0 0; 0 0 -1; 0 1 0];          % Q2.2
R2 = [0.5 -sqrt(3)/2 0; sqrt(3)/2 0.5 0; 0 0 1];  % Q2.3
R3 = [1 0 0; 0 1 0; 0 0 1];            % Q2.4
R4 = [-1 0 0; 0 -1 0; 0 0 1];           % Q2.5

% Apply the function to each matrix and display results
matrices = {R1, R2, R3, R4};
for i = 1:length(matrices)
    try
        [h, theta] = RotToAngleAxis(matrices{i});
        fprintf('Matrix Q2.%d:\n', i+1);
        fprintf('Axis h: [%.4f, %.4f, %.4f]\n', h);
        fprintf('Angle theta: %.4f degrees\n\n', theta);
    catch ME
        fprintf('Matrix Q2.%d:\n', i+1);
        fprintf('Error: %s\n\n', ME.message);
    end
end

% 1.3 Euler to rot
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

% 1.4 Rot to Euler
R1 = [1 0 0; 0 0 -1; 0 1 0];
R2 = [1/2 -sqrt(3)/2 0; sqrt(3)/2 1/2 0; 0 0 1];
R3 = [0 -sqrt(2)/2 sqrt(2)/2; 0.5 sqrt(2)*sqrt(3)/4 -sqrt(2)/4; -sqrt(3)/2 sqrt(2)/4 sqrt(2)/4];

% Store the matrices in a cell array for easy iteration
rotation_matrices = {R1, R2, R3};

% Iterate over each matrix and compute YPR angles
for i = 1:length(rotation_matrices)
    R = rotation_matrices{i};
    [psi, theta, phi] = RotToYPR(R);
    fprintf('For R%d:\n', i);
    fprintf('Psi (Yaw) = %.4f, Theta (Pitch) = %.4f, Phi (Roll) = %.4f\n\n', psi, theta, phi);
end