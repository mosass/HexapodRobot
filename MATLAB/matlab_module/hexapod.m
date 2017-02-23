classdef hexapod
    %HEXAPOD Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % length of leg links
        link_length = [4.5 6.5 6.5];
        % link_offset = [-pi/2 atan(2/6) atan(2/6)-pi/2];
        link_offset = [0 atan(2/6) -atan(2/6)+pi/2];
        % link_offset = [0 0 0];
        % body size
        body = [15 30];
        %COM Port
        portname = 'COM6';
        baudrate = 115200;
    end
    
    methods
        function r = leg(obj)
            L1 = obj.link_length(1);
            L2 = obj.link_length(2);
            L3 = obj.link_length(3);
            % create the leg links based on DH parameters
            %                    theta   d     a  alpha
            links(1) = Link([    0       0    L1    pi/2], 'standard');
            links(2) = Link([    0       0    L2      0 ], 'standard');
            links(3) = Link([    0       0    L3      0 ], 'standard');

            % now create a robot to represent a single leg
            r = SerialLink(links, 'name', 'leg', ... 
                 'offset', obj.link_offset);
        end
        
        function r = legs4l(obj)
            leg = obj.leg();
            legs(1) = SerialLink(leg, 'name', 'leg1', 'base', transl(5, 0, 0)*trotz(-pi/2)*trotx(pi));
            legs(2) = SerialLink(leg, 'name', 'leg2', 'base', transl(25, 0, 0)*trotz(-pi/2)*trotx(pi));
            % legs(5) = SerialLink(leg, 'name', 'leg5', 'base', transl(15, 0, 0)*trotz(-pi/2)*trotx(pi));
            legs(3) = SerialLink(leg, 'name', 'leg3', 'base', transl(5, 15, 0)*trotz(pi/2)*trotx(pi));
            legs(4) = SerialLink(leg, 'name', 'leg4', 'base', transl(25, 15, 0)*trotz(pi/2)*trotx(pi));
            % legs(6) = SerialLink(leg, 'name', 'leg6', 'base', transl(15, 15, 0)*trotz(pi/2)*trotx(pi));
            
            r = legs;
        end
        
        function simulate4l(obj, qcycle, q_offset, n_round, q_res)
            plotopt = {'noraise', 'nobase', 'noshadow', 'nowrist', 'nojaxes', 'delay', 0};
            
            legs = obj.legs4l();
            w = obj.body(1);
            l = obj.body(2);

            body_x = [0  l  l  0];
            body_y = [0  0  w  w];
            body_z = [0  0  0  0];

            clf;
            axis([-5 35 -10 30 -15 5]);
            hold on
            patch(body_x, body_y, body_z, 'FaceColor', 'r', 'FaceAlpha', 1)
            for i=1:4
                legs(i).plot(qcycle(1,:), plotopt{:});
            end
            hold off
            
            [M N] = size(qcycle);
            
            %A = Animate('walking');
            for j=1:n_round
                for i=1:q_res:M
                    idx = [i i i i] + (q_offset.*[0 1 2 3]) - 1;
                    idx = mod(idx, M) + 1;
                    
                    q1 = qcycle(idx(1), :);
                    q2 = qcycle(idx(2), :);
                    q3 = qcycle(idx(3), :);
                    q4 = qcycle(idx(4), :);
                    
                    legs(1).animate(q1);
                    legs(2).animate(q2);
                    
                    q3(1) = -q3(1);
                    q4(1) = -q4(1);
                    legs(3).animate(q3);
                    legs(4).animate(q4);
                    drawnow
                    %A.add();
                end
            end
        end
        
        function genArrayC(obj, qcycle, filename)
            qcycle = rad2deg(qcycle) + 150;
            [M N] = size(qcycle);
            
            fd = fopen(filename,'wt');
            
            fprintf(fd, 'float QCYCLE[%d][%d] = {', M, N);
            for i = 1:M-1
                fprintf(fd, '\n\t{%.9f,\t%.9f,\t%.9f},', qcycle(i, :));
            end
            
            fprintf(fd, '\n\t{%.9f,\t%.9f,\t%.9f}', qcycle(M, :));
            fprintf(fd,'\n};\n');
            fclose(fd);
        end
        
        function [q, s] = q2pos4l(obj, qcycle, q_offset, dt)
            qcycle = rad2deg(qcycle);
            [M N] = size(qcycle);
            q = [];
            for i=1:M
                idx = [i i i i] + (q_offset.*[0 1 2 3]) - 1;
                idx = mod(idx, M) + 1;

                q(i, :) = [
                    qcycle(idx(1), :) ...
                    qcycle(idx(2), :) ...
                    qcycle(idx(3), :) ...
                    qcycle(idx(4), :)
                ];
                % *(-1) for left-side legs
                q(i, :) = q(i, :).*[1 1 1  1 1 1  -1 1 1  -1 1 1];
            end
            
            [M N] = size(q);

            shift_q = [q(2:M, :); q(1, :)];
            dq = abs(q - shift_q);
            
            s = ceil((dq./dt)/0.666)+1;
%             s = mod(s, 500);
            q = 511 + round(q./0.29);
        end
        
        function send_command(obj, q, s, dt, n_round, q_res)
            FID = serial(obj.portname);
            set(FID,'BaudRate', obj.baudrate);
            fopen(FID);
            
            [M N] = size(q);
            for k = 1:n_round
                for j = 1:q_res:M
                    for i = 1:N
                        send_packet(FID, i, q(j, i), s(j, i));
                        pause(dt);
                    end
                end
            end
            
            fclose(FID);
            delete(FID);
            clear FID;
        end
    end
end

