% cannon_ball_kalman.m
% written by Syed Zeeshan Akhtar (email: akhtar.syedzeeshan@gmail.com )
%
% Implements a multi-variable linear Kalman filter.
%
% References:
%
% 1) This code is based on a tutorial "Kalman Filters for Undergrads"
% located at http://greg.czerniak.info/node/5.

 
%=============================REAL PROGRAM START================================
% Let's go over the physics behind the cannon shot, just to make sure it's
% correct:
% sin(45)*100 = 70.710 and cos(45)*100 = 70.710
% vf = vo + at
% 0 = 70.710 + (-9.81)t
% t = 70.710/9.81 = 7.208 seconds for half
% 14.416 seconds for full journey
% distance = 70.710 m/s * 14.416 sec = 1019.36796 m

timeslice = 0.1; % How many seconds should elapse per iteration?
iterations = 144; % How many iterations should the simulation run for?
% (notice that the full journey takes 14.416 seconds, so 145 iterations will
% cover the whole thing when timeslice = 0.10)

gravity = [0,-9.81];
wind = [0,0];
noiselevel = 5;  % How much noise should we add to the noisy measurements?
muzzle_velocity = 100; % How fast should the cannonball come out?
clc;

angle = 45; % Angle from the ground.

% These are arrays to store the data points we want to plot at the end.
x = zeros(iterations);
y = zeros(iterations);
nx = zeros(iterations);
ny = zeros(iterations);
kx = zeros(iterations);
ky = zeros(iterations);


% Let's make a cannon simulation.
c = Cannon(angle, muzzle_velocity, gravity, wind, timeslice,noiselevel)

speedX = 1000; %muzzle_velocity*cosd(angle);
speedY = 1000; %muzzle_velocity*sind(angle);

% This is the state transition vector, which represents part of the kinematics.
% 1, ts, 0,  0  =>  x(n+1) = x(n) + vx(n)
% 0,  1, 0,  0  => vx(n+1) =        vx(n)
% 0,  0, 1, ts  =>  y(n+1) =              y(n) + vy(n)
% 0,  0, 0,  1  => vy(n+1) =                     vy(n)
% Remember, acceleration gets added to these at the control vector.
state_transition = [1,timeslice,0,0; 0,1,0,0; 0,0,1,timeslice; 0,0,0,1]

control_matrix = [0,0,0,0; 0,0,0,0; 0,0,1,0; 0,0,0,1]
% The control vector, which adds acceleration to the kinematic equations.
% 0          =>  x(n+1) =  x(n+1)
% 0          => vx(n+1) = vx(n+1)
% -9.81*ts^2 =>  y(n+1) =  y(n+1) + 0.5*-9.81*ts^2
% -9.81*ts   => vy(n+1) = vy(n+1) + -9.81*ts
control_vector = [0; 0; 0.5*gravity(1,2)*timeslice*timeslice; gravity(1,2)*timeslice]

% After state transition and control, here are the equations:
%  x(n+1) = x(n) + vx(n)
% vx(n+1) = vx(n)
%  y(n+1) = y(n) + vy(n) - 0.5*9.81*ts^2
% vy(n+1) = vy(n) + -9.81*ts
% Which, if you recall, are the equations of motion for a parabola.  Perfect.

% Observation matrix is the identity matrix, since we can get direct
% measurements of all values in our example.
observation_matrix = eye(4)

% This is our guess of the initial state.  I intentionally set the Y value
% wrong to illustrate how fast the Kalman filter will pick up on that.
initial_state = [0; speedX; 300; speedY]

initial_probability = eye(4)

process_covariance = zeros(4)
measurement_covariance = eye(4)*0.2

kf = KalmanFilterLinear(state_transition, control_matrix, observation_matrix, initial_state, initial_probability, process_covariance, measurement_covariance)

% Iterate through the simulation.
for i = 1: iterations
    x(i) = c.GetX();
    y(i) = c.GetY();
    nx(i) = c.GetXWithNoise();
    ny(i) = c.GetYWithNoise();
    % Iterate the cannon simulation to the next timeslice.
    
    c.Step();
    cur_state = kf.GetCurrentState()
    kx(i) = cur_state(1,1);
    ky(i) = cur_state(3,1);
    measurement_vector = [nx(i);c.GetXVelocity();ny(i);c.GetYVelocity()];
    kf.Step(control_vector, measurement_vector);
end

% Plot all the results we got.
figure(1);
subplot(111);
hold off
plot(x,y,'g-',nx,ny,'r-',kx,ky,'b-');
xlabel('X position');
ylabel('Y position');
title('Measurement of a Cannonball in Flight with 4 times noise level');
