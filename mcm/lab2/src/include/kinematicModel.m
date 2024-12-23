%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end
        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix

            % TO DO
            % fprintf('joint type array');
            % disp(self.gm.jointType);
            % fprintf('\narray length: %d\n',length(self.gm.jointType));
            bTe = self.gm.getTransformWrtBase(length(self.gm.jointType));
            for i = 1:length(self.gm.jointType)
                jointType = self.gm.jointType(i);
                T = self.gm.getTransformWrtBase(i);
                
                % Calculate position of end-effector
                p_e = bTe(1:3, 4, end);
                p_i = T(1:3, 4);
                
                r = p_e - p_i; % Vector from joint i to end-effector
                
                R = T(1:3, 1:3); 
               
                z_i = R(:, 3); % z-axis of the rotation matrix for revolute joints
                
                if (jointType == 0) % Revolute joint
                    % Angular velocity: z_i (axis of rotation)
                    self.J(1:3, i) = z_i;
                    % Linear velocity: cross-product of z_i and r
                    self.J(4:6, i) = cross(z_i, r); 
                elseif (jointType == 1) % Prismatic joint
                    % Angular velocity: 0 for prismatic joint
                    self.J(1:3, i) = [0; 0; 0]; 
                    % Linear velocity: direction of translation (same as z-axis of R)
                    self.J(4:6, i) = R(:, 3); 
                end
            end

            
        end
    end
end

