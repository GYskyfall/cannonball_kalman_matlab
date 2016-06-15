% Implements a linear Kalman filter.
classdef KalmanFilterLinear < handle
  properties
    A                      % State transition matrix.
    B                      % Control matrix.
    H                      % Observation matrix.
    current_state_estimate % Initial state estimate.
    current_prob_estimate  % Initial covariance estimate.
    Q                      % Estimated error in process.
    R                      % Estimated error in measurements.  
  end
  methods
        function obj = KalmanFilterLinear(a, b, h, x, p, q, r)
            obj.A = a;                      % State transition matrix.
            obj.B = b;                      % Control matrix.
            obj.H = h;                      % Observation matrix.
            obj.current_state_estimate = x; % Initial state estimate.
            obj.current_prob_estimate = p;  % Initial covariance estimate.
            obj.Q = q;                      % Estimated error in process.
            obj.R = r;                      % Estimated error in measurements.
        end
        
        function r = GetCurrentState(obj)
            r= obj.current_state_estimate;
        end
        function Step(obj, control_vector, measurement_vector)
            %---------------------------Prediction step-----------------------------
            predicted_state_estimate = obj.A * obj.current_state_estimate + obj.B * control_vector;
            predicted_prob_estimate = (obj.A * obj.current_prob_estimate) * transpose(obj.A) + obj.Q;
            %--------------------------Observation step-----------------------------
            innovation = measurement_vector - obj.H*predicted_state_estimate;
            innovation_covariance = obj.H*predicted_prob_estimate*transpose(obj.H) + obj.R;
            %-----------------------------Update step-------------------------------
            kalman_gain = predicted_prob_estimate * transpose(obj.H) * inv(innovation_covariance);
            obj.current_state_estimate = predicted_state_estimate + kalman_gain * innovation;
            % We need the size of the matrix so we can make an identity matrix.
            size_est = size(obj.current_prob_estimate);
            % eye(n) = nxn identity matrix.
            obj.current_prob_estimate = (eye(size_est(1))-kalman_gain*obj.H)*predicted_prob_estimate;
            
        end
% Simulates the classic physics problem of a cannon shooting a ball in a
% parabolic arc.  In addition to giving "true" values back, you can also ask
% for noisy values back to test Kalman filters.
  end
end

