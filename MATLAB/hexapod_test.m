close all
clear all

% startup_rtb
robot = hexapod();

robot.plot();

%       T   x   y   z   r   p   y
seg = [ 1   0   0   0   0   0   0;
        1   0   0   4   0   0   0; 
        1   0   0   -3  0   0   0;
        1   3   0   -3  0   0   0;
        1   3   3   -3  0   0   0;
        1   0   3   -3  0   0   0;
        1   -3  3   -3  0   0   0;
        1   -3  0   -3  0   0   0;
        1   -3  -3  -3  0   0   0;
        1   0   -3  -3  0   0   0;
        1   3   -3  -3  0   0   0;
      ];


traj = mstraj(seg(:,2:7), [], seg(:,1)', seg(1,2:7), 0.3, 0.1);

[M N] = size(traj);
for i = 1:M
    robot.setBodyTransl(traj(i, :));
    robot.animate();
end


robot.setBodyTransl([0 0 0 5 0 0]);
robot.animate();



