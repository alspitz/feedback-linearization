%
% This implements MIMO feedback linearization
% using dynamic extension for the 2D quadrotor with the state
%
% x = [ position, velocity, theta, angular velocity ]
%
% and the control
%
% u = [ thrust, angular acceleration ].
%
% The augmented state becomes
%
% x = [ pos, vel, theta, om, u, udot ].

function test_posquad
  g = 9.8;

  p0 = [8 0];
  v0 = [0 0];

  x0 = [ p0 v0 0 0 g 0 ]';
  [ts, xs] = ode45(@f, [0 5], x0);

  figure
  subplot(211)
  plot(ts, xs(:, 1))
  ylabel('p_x')
  xlabel('time')
  subplot(212)
  plot(ts, xs(:, 2))
  ylabel('p_z')
  xlabel('time')

  figure
  subplot(211)
  plot(ts, xs(:, 3))
  ylabel('v_x')
  xlabel('time')
  subplot(212)
  plot(ts, xs(:, 4))
  ylabel('v_z')
  xlabel('time')

  figure
  subplot(211)
  plot(ts, xs(:, 5))
  ylabel('\theta')
  xlabel('time')
  subplot(212)
  plot(ts, xs(:, 6))
  ylabel('\omega')
  xlabel('time')

  figure
  subplot(211)
  plot(ts, xs(:, 7))
  ylabel('u')
  xlabel('time')
  subplot(212)
  plot(ts, xs(:, 8))
  ylabel('u dot')
  xlabel('time')
end

function dx = f(t, x)
  g = 9.8;
  p = x(1:2);
  v = x(3:4);
  theta = x(5);
  om = x(6);
  z1 = x(7);
  z2 = x(8);

  z = [-sin(theta) cos(theta)]';
  zdot = [ -cos(theta) * om -sin(theta) * om ]';

  a = z1 * z - [0 g]';
  j = z2 * z + z1 * zdot;

  Kp = 840 * eye(2);
  Kv = 480 * eye(2);
  Ka = 120 * eye(2);
  Kj = 16 * eye(2);

  snap = -Kp * p - Kv * v - Ka * a - Kj * j;

  v1 = snap' * z + z1 * zdot' * zdot;

  zddot = (1 / z1) * (snap - v1 * z - 2 * z2 * zdot);

  v2 = cross([z' 0], [zddot' 0]);
  v2 = v2(3);

  dx = [v' a' om v2 z2 v1]';
end
