function output = TwoComp_MKO(x)
    temp = cast(x, 'uint16');
    temp = bitshift(temp, 4);
    temp = typecast(temp(:), 'int16');
    temp = bitshift(temp, -4, 'int16');
    temp = cast(temp, 'double');
    
    output = reshape(temp, size(x,1), size(x,2));
end