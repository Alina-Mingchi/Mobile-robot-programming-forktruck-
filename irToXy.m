function [x y th] = irToXy(i,r)
%irToXy finds position and bearing of a range pixel endpoint
% Finds the position and bearing of the endpoint of a range pixel in the
% plane

%robot frame x forward, y to the left
%bearing [-pi pi], ccw positive
% Offset = atan2(0.024,0.28);
Offset = 0;
th = (i-1)*pi/180-Offset; %rad
x = r .* cos(th);
y = r .* sin(th);

end



