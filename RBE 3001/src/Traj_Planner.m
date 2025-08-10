% (c) 2023 Robotics Engineering Department, WPI
% Trajectory Planner class for RBE 3001

classdef Traj_Planner
    % Doesnt need Any :D
    properties
    end

    methods
        
        function self = Traj_Planner()
           
            
            
        end
        
        %Calculates the 4x4 cubicTrajectory Matrix and then multiples it by
        %4x1 so that the currentJoint Positions need to follow trajector
        %are outputted as a 4x1
        %Takes in Time Initial, time Final, velocity initial, velocity
        %final, position initial, position final
        function T = cubic_traj(self, t0, tf, v0, vf, q0, qf)
            M = [1 , t0, t0^2, t0^3;    
            0, 1, 2*t0, 3*t0^2;
            1, tf, tf^2, tf^3;
            0, 1, 2*tf, 3*tf^2];
            B = [q0; v0; qf; vf];
            T = inv(M)*B;
        end 
        
        %Calculates the 6x6 quinticTrajectory Matrix and then multiples it by
        %6x1 so that the currentJoint Positions need to follow trajector
        %are outputted as a 6x1
        %Takes in Time Initial, time Final, velocity initial, velocity
        %final, position initial, position final, acceleration initial,
        %acceleration fianl
        function T = quintic_traj(self, t0, tf, v0, vf, q0, qf, alpha0, alphaf)
        M = [
          1, t0, t0^2, t0^3,t0^4, t0^5;
          0, 1, 2*t0, 3*t0^3, 4*t0^3, 5*t0^4;
          0, 0, 2, 6*t0, 12*t0^2, 20*t0^3;
          1, tf, tf^2, tf^3, tf^4, tf^5;
          0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
          0, 0, 2, 6*tf, 12*tf^2, 20*tf^3        
        ];
        B =[q0; v0; alpha0; qf; vf; alphaf];
        T = inv(M)*B;
        end
        
        
    end % end methods
end % end class 
