function send_packet(FID, id, position, speed)
    data = repmat(uint8(0),[1 5]);
    data(1) = id;
    data(2) = bitshift(position, -8);
    data(3) = bitand(position, 255);
    data(4) = bitshift(speed, -8);
    data(5) = bitand(speed, 255);
    fwrite(FID, data);
end