classdef Cannon < handle
    properties
      %--------------------------------VARIABLES----------------------------------
      angle % The angle from the ground to point the cannon.
      muzzle_velocity % Muzzle velocity of the cannon.
      gravity % A vector containing gravitational acceleration.
      wind % A vector containing wind force
      velocity % The initial velocity of the cannonball
      loc % The initial location of the cannonball.
      acceleration % The initial acceleration of the cannonball.
      timeslice
      noiselevel
    end
      %---------------------------------METHODS-----------------------------------
  methods
      function obj = Cannon(angle, muzzle_velocity, gravity, wind, time_slice,noise_level)
        obj.timeslice = time_slice;
        obj.noiselevel = noise_level;
        obj.angle = angle;
        obj.muzzle_velocity = muzzle_velocity;
        obj.gravity = gravity;
        obj.wind = wind;
        obj.velocity = [obj.muzzle_velocity*cos(obj.angle*pi/180), obj.muzzle_velocity*sin(obj.angle*pi/180)];
        obj.loc = [0 0]; % The initial location of the cannonball.
        obj.acceleration = [0 0]; % The initial acceleration of the cannonball.
      end
      function r = GetX(obj)
        r = obj.loc(1, 1);
      end
      function r = GetY(obj)
        r = obj.loc(1, 2);
      end
      function r = GetXWithNoise(obj)
        r = normrnd(obj.GetX(),obj.noiselevel);
      end
      function r = GetYWithNoise(obj)
        r = normrnd(obj.GetY(),obj.noiselevel);
      end
      function r = GetXVelocity(obj)
        r = obj.velocity(1, 1);
      end
      function r = GetYVelocity(obj)
        r = obj.velocity(1, 2);
      end
  
      % Increment through the next timeslice of the simulation.
      function Step(obj)
        % We're gonna use this vector to timeslice everything.
        %timeslicevec = [obj.timeslice;obj.timeslice];
        
        % Break gravitational force into a smaller time slice.
        sliced_gravity = [obj.gravity(1,1)*obj.timeslice obj.gravity(1,2)*obj.timeslice];
        
        % Break wind force into a smaller time slice.
        sliced_wind = [obj.wind(1,1)*obj.timeslice obj.wind(1,2)*obj.timeslice];
        
        % The only force on the cannonball is gravity.
        sliced_acceleration = sliced_gravity + sliced_wind;
        
        % Apply the acceleration to velocity.
        obj.velocity = [obj.velocity(1,1)+sliced_acceleration(1,1) obj.velocity(1,2)+sliced_acceleration(1,2)];
        
        % Break velocity into a smaller time slice.
        sliced_velocity = [obj.velocity(1,1)*obj.timeslice obj.velocity(1,2)*obj.timeslice];
        
        % Apply the time sliced velocity to location.
        obj.loc = [obj.loc(1,1)+sliced_velocity(1,1) obj.loc(1,2)+sliced_velocity(1,2)];
        
        % Cannonballs shouldn't go into the ground.
        if obj.loc(1,2) < 0
          obj.loc(1,2) = 0;
        end
      end
  end
end
