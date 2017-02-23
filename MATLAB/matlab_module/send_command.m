function res = send_command(q, dt)
    portname = 'COM7';
    baudrate = 115200;
    
    ser = serial(portname);
    set(ser,'BaudRate', baudrate);
    fopen(ser);
    
    q = rad2deg(q);
    
    [M N] = size(q);
    
    shift_q = [q(2:M, :); q(1, :)];
    dq = abs(q - shift_q);
    speed = ceil((dq./dt)/0.666);
    
    pos = 511 + round(q./0.29);
    
    send_packet(ser, 1, pos(1, 1), speed(1, 1));
    send_packet(ser, 2, pos(1, 2), speed(1, 2));
    send_packet(ser, 3, pos(1, 3), speed(1, 3));
    
    for i = 2:M
        send_packet(ser, 1, pos(i, 1), speed(i, 1));
        send_packet(ser, 2, pos(i, 2), speed(i, 2));
        send_packet(ser, 3, pos(i, 3), speed(i, 3));
        
        pause(dt);
    end
    
    fclose(ser);
    delete(ser);
    clear ser;
    res = 'OK';
end

