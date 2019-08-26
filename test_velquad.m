%
% This implements MIMO feedback linearization
% using dynamic extension for the 2D quadrotor with the state
%
% x = [ velocity, theta ]
%
% and the control
%
% u = [ thrust, angular velocity ].
%
% The augmented state becomes
%
% x = [ velocity, theta, u ].

function test_velquad
  g = 9.8;

  x0 = [ 20 0 0 g ]';
  [ts, xs] = ode45(@f, [0 5], x0);

  subplot(411)
  plot(ts, xs(:, 1))
  ylabel('v_x')
  xlabel('time')

  subplot(412)
  plot(ts, xs(:, 2))
  ylabel('v_z')
  xlabel('time')

  subplot(413)
  plot(ts, xs(:, 3))
  ylabel('\theta')
  xlabel('time')

  subplot(414)
  plot(ts, xs(:, 4))
  ylabel('u')
  xlabel('time')
end

function dx = f(t, x)
  g = 9.8;
  v = x(1:2);
  theta = x(3);
  z1 = x(4);

  z = [-sin(theta) cos(theta)]';

  a = z1 * z - [0 g]';

  Kp = 6 * eye(2);
  Kd = 4 * eye(2);

  j = -Kp * v -Kd * a;

  v1 = j' * z;

  zdot = (1 / z1) * (j - v1 * z);

  v2 = cross([z' 0], [zdot' 0]);
  v2 = v2(3);

  dx = [a' v2 v1]';
end
