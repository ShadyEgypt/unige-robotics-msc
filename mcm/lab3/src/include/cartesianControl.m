%% Kinematic Model Class - GRAAL Lab
classdef cartesianControl < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        k_a
        k_l
    end

    methods
        % Constructor to initialize the geomModel property
        function self = cartesianControl(gm,angular_gain,linear_gain)
            if nargin > 2
                self.gm = gm;
                self.k_a = angular_gain;
                self.k_l = linear_gain;
            else
                error('Not enough input arguments (cartesianControl)')
            end
        end
        function [x_dot]=getCartesianReference(self,bTg)
            %% getCartesianReference function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control
            bTt = self.gm.getToolTransformWrtBase();
            cartesian_error = bTg(1:3,4) - bTt(1:3,4);
            disp('cartesian error');
            disp(cartesian_error);
            tTg = bTt \ bTg; 
            [h, theta] = RotToAngleAxis(tTg(1:3,1:3));
            angular_error = bTt(1:3,1:3) * h * theta;
            disp('angular error');
            disp(angular_error);
            error = [angular_error ; cartesian_error];

            K = [self.k_a * eye(3) zeros(3); zeros(3) self.k_l * eye(3)];
            disp('error');
            disp(error);
            disp('k')
            disp(K);
            x_dot = K * error;
       
            
        end
    end
end