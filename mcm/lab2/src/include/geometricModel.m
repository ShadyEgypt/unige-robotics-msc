%% Geometric Model Class - GRAAL Lab
classdef geometricModel < handle
    % iTj_0 is an object containing the trasformations from the frame <i> to <i'> which
    % for q = 0 is equal to the trasformation from <i> to <i+1> = >j>
    % (see notes)
    % jointType is a vector containing the type of the i-th joint (0 rotation, 1 prismatic)
    % jointNumber is a int and correspond to the number of joints
    % q is a given configuration of the joints
    % iTj is  vector of matrices containing the transformation matrices from link i to link j for the input q.
    % The size of iTj is equal to (4,4,numberOfLinks)
    properties
        iTj_0
        jointType
        jointNumber
        iTj
        q
    end

    methods
        % Constructor to initialize the geomModel property
        function self = geometricModel(iTj_0,jointType)
            if nargin > 1
                self.iTj_0 = iTj_0;
                self.iTj = iTj_0;
                self.jointType = jointType;
                self.jointNumber = length(jointType);
                self.q = zeros(self.jointNumber,1);
            else
                error('Not enough input arguments (iTj_0) (jointType)')
            end
        end
        function updateDirectGeometry(self, q)
            % Update the iTj transformation matrices based on the joint positions q
        
            % Step 1: Validate the input dimensions
            if length(q) ~= self.jointNumber
                error('The input q must have the same length as the number of joints.');
            end
        
            % Step 2: Update the joint positions
            self.q = q;
        
            % Step 3: Initialize iTj using iTj_0 as the base transformation for q = 0
            self.iTj = self.iTj_0;
        
            % Step 4: Iterate through each joint and update the transformation matrix
            for i = 1:self.jointNumber
                % Extract the joint type (0 for rotational, 1 for prismatic)
                jointType = self.jointType(i);
            
                % Get the joint transformation matrix for the current joint position q(i)
                if jointType == 0 % Rotational joint
                    % Apply rotation about the z-axis by angle q(i)
                    jointTransform = [cos(q(i)), -sin(q(i)), 0, 0;
                                      sin(q(i)),  cos(q(i)), 0, 0;
                                      0,          0,         1, 0;
                                      0,          0,         0, 1];
                elseif jointType == 1 % Prismatic joint
                    % Apply translation along the z-axis by distance q(i)
                    jointTransform = [1, 0, 0, 0;
                                      0, 1, 0, 0;
                                      0, 0, 1, q(i);
                                      0, 0, 0, 1];
                else
                    error('Invalid joint type. Joint type must be 0 (rotational) or 1 (prismatic).');
                end
        
                % Compute the cross product for the i-th joint
                iTj = self.iTj(:, :, i);
                new_iTj = iTj * jointTransform;
                self.iTj(:, :, i) = new_iTj;
            
                % % Display iTj, jointTransform, and their product on one line
                % fprintf('Joint %d:\n', i);
                % fprintf('[iTj] * [jointTransform] = [resulting values]\n');
                % fprintf('iTj:\n');
                % disp(iTj);
                % fprintf('jointTransform:\n');
                % disp(jointTransform);
                % fprintf('Result:\n');
                % disp(new_iTj);
            end
         end    
        function [bTk] = getTransformWrtBase(self, k)
            %% GetTransformationWrtBase function
            % Inputs:
            % k: the index of the joint for which the transformation matrix is to be computed
            % Outputs:
            % bTk: transformation matrix from the manipulator base to the k-th joint
        
            % Initialize the transformation matrix as the identity matrix
            bTk = eye(4);
        
            % Loop through all joints from the base to the k-th joint
            for i = 1:k
                % Multiply the transformation matrices to accumulate the result
                bTk = bTk * self.iTj(:, :, i);
            end
        end

    end
end