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
    % eTt (OPTIONAL) add a tool to the model rigid attached to the
    % end-effector
    properties
        iTj_0
        jointType
        jointNumber
        iTj
        q
        eTt
    end

    methods
        % Constructor to initialize the geomModel property
        function self = geometricModel(iTj_0,jointType,eTt)
            if nargin > 1
                 if ~exist('eTt','var')
                     % third parameter does not exist, so default it to something
                      eTt = eye(4);
                 end
                self.iTj_0 = iTj_0;
                self.iTj = iTj_0;
                self.jointType = jointType;
                self.jointNumber = length(jointType);
                self.q = zeros(self.jointNumber,1);
                self.eTt = eTt;
            else
                error('Not enough input arguments (iTj_0) (jointType)')
            end
        end
        function updateDirectGeometry(self, q)
            %%% GetDirectGeometryFunction
            % This method update the matrices iTj.
            % Inputs:
            % q : joints current position ;

            % The function updates:
            % - iTj: vector of matrices containing the transformation matrices from link i to link j for the input q.
            % The size of iTj is equal to (4,4,numberOfLinks)
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
            end
        end
        function [bTk] = getTransformWrtBase(self,k)
            %% GetTransformatioWrtBase function
            % Inputs :
            % k: the idx for which computing the transformation matrix
            % outputs
            % bTk : transformation matrix from the manipulator base to the k-th joint in
            % the configuration identified by iTj.
            % Initialize the transformation matrix as the identity matrix
            bTk = eye(4);
        
            % Loop through all joints from the base to the k-th joint
            for i = 1:k
                % Multiply the transformation matrices to accumulate the result
                bTk = bTk * self.iTj(:, :, i);
            end
        end
        function [bTt] = getToolTransformWrtBase(self)
            %% getToolTransformWrtBase function
            % Inputs :
            % None 
            % bTt : transformation matrix from the manipulator base to the
            % tool
            bTe = self.getTransformWrtBase(self.jointNumber);
            bTt = bTe * self.eTt;
        end
    end
end


