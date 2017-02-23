close all
clear all

% startup_rtb
robot = hexapod();
leg = robot.leg();

yf = 4; yb = -yf;   % forward and backward limits for foot on ground
x = 8;              % distance of foot from body along y-axis
zu = 7; zd = 11    % height of foot when up and down
% define the rectangular path taken by the foot
segments = [x yf zd; x yb zd; x yb zu; x yf zu];

segments = [segments; segments];
tseg = [3 0.25 0.5 0.25]';
tseg = [1; tseg; tseg];
x = mstraj(segments, [], tseg, segments(1,:), 0.01, 0.1);

% plot(x);
% pull out the cycle
xcycle = x(100:500,:);

qcycle = leg.ikine( transl(xcycle), [], [1 1 1 0 0 0]);
[M N] = size(qcycle);
plot(rad2deg(qcycle) + 150);

robot.genArrayC(qcycle, 'qcycle.h');

offset = 100;
dt = 0.007;

% robot.simulate4l(qcycle, offset, 1, 5);
robot.simulate4l(qcycle, offset, 1, 1);
axis([-5 35 -20 35 -10 5]);

% [pos, speed] = robot.q2pos4l(qcycle, offset, dt);
% max(speed)
% min(speed)

q = rad2deg(qcycle(:,3));
plot([q]);
hold on;
M = length(q);

overQ = [];
diffQ = [];
for idx = 1:M
    pre_idx = idx - 1;
    if (pre_idx == 0)
        pre_idx = M;
    end
    diffQ(idx) = (q(idx) - q(pre_idx)) * 5;
    overQ(idx) = q(idx) + diffQ(idx);
end
plot([overQ]);
hold off;
