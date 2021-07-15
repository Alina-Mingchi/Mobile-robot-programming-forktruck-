classdef rangeImage < handle
    
    properties
        robot
    end
    
    methods
        function obj = rangeImage(robot)
            obj.robot = robot
            
        end
        
        function [x,y,b,readings] = noise_cancellation(obj,robot)
            global encoder_pose
            t = true;
            blub = false;
            while t
                
                read = obj.robot.laser.LatestMessage.Ranges;
                pause(1);
                r = read';
                i = 1:360;
                
                newRangeImage = removeBadPoints(obj,r);
                [x y b] = irToXy(obj,i,newRangeImage);
%                 bgt = find((encoder_pose(1)-x)<0)
%                 if bgt(1) == 1 || bgt(2) == 2
%                     blub = true;
%                 end
                if max(x)< 0.2 || max(y)< 0.2
                    read = robot.laser.LatestMessage.Ranges;
                    pause(1);
                    r = read';
                    i = 1:360
                    
                    newRangeImage = removeBadPoints(obj,r);
                    [x y b] = irToXy(obj,i,newRangeImage);
                elseif blub == true
                    read = robot.laser.LatestMessage.Ranges;
                    pause(1);
                    r = read';
                    i = 1:360
                    
                    newRangeImage = removeBadPoints(obj,r);
                    [x y b] = irToXy(obj,i,newRangeImage);
                else
                    t = false;
                    readings = r;
                end
            end
        end
            
            
            
        function [x y th] = irToXy(obj,i,rangeImg)
            
            Offset = atan2(0.024,0.28);
            
            th = (i-1)*pi/180-Offset; %rad
            x = rangeImg .* cos(th);
            y = rangeImg .* sin(th);
            
        end
        
        function [rangeImg] = removeBadPoints(obj, rangeImg)
            
            for i=1:360
                if rangeImg(i) < 0.06 || rangeImg(i) > 2.0
                    rangeImg(i) = 0;
                end
            end
            
        end
        
        function th= correct_theta(obj,pth)
            th = pth*180/pi;
            for i=1:size(th)
                if th(i)>180
                    th(i) = th(i) - 360;
                end
            end
            
            
        end
        
        
        
        function [px, py, pth] = findLineCandidate(obj,r)
            global angle;
            global top;
            global angle_2;
            pause(1)
            rangeImg = r;
            a = 1;
            b = 1;
            Offset = atan2(0.024,0.28);
            goodOnes = rangeImg > 0.06 & rangeImg < 2.0;
            rangeImg = rangeImg(goodOnes);
            indices = 1:360;
            indices = indices';
            indices = indices(goodOnes);
            % Px,Py and Pth are all the positons of Good Points
            pth = (indices-1)*(pi/180)-Offset;
            px = rangeImg' .* cos(pth);
            py = rangeImg' .* sin(pth);
            
            validpt = size(indices,1);
            range = 0.11;
            
            
            for count = 1:size(indices,1)
                
                Offset = atan2(0.024,0.28);
                th = (indices(count)-1)*pi/180-Offset; %rad
                x = r(indices(count))* cos(th);
                y = r(indices(count))* sin(th);
                figure(1)
                temp = sqrt((px-x).^2 + (py-y).^2);
                
                pot_centroidx(count,:) = mean(px(temp<range));
                pot_centroidy(count,:) = mean(py(temp<range));
                
                xx = px(temp<range) - mean(px(temp<range));
                yy = py(temp<range) - mean(py(temp<range));
                
                
                
                Ixx = sum(xx .* xx);
                Iyy = sum(yy .* yy);
                Ixy = - sum(xx .* yy);
                Inertia = [Ixx Ixy;Ixy Iyy] / size(px(temp<range),1); % normalized
                lambda = eig(Inertia);
                lambda = sqrt(lambda)*1000.0;
                
                
                if lambda(1,1) < 1.5
                    centroidx(a,:) = pot_centroidx(count,:);
                    centroidy(a,:) = pot_centroidy(count,:);
                    theta(a,:) = atan2(2*Ixy,Iyy-Ixx)/2.0;
                    try_theta(a,:) = pth(count,:);
%                     plot(centroidx(a,:),centroidy(a,:),'*r');
%                     hold on
%                     axis([-3 3 -3 3]);
                    a = a+1;
                end
                
            end
            try_theta= correct_theta(obj,try_theta);
            %Removing duplicates
            %centroidx = unique(centroidx,'stable');
            %centroidy = unique(centroidy,'stable');
            centroid = sqrt((centroidx).^2 + (centroidy).^2);
            
            
            
            
            
            for count = 2:size(centroidx,1)
                temp2 = sqrt((centroidx(count,:)-centroidx).^2 + (centroidy(count,:)-centroidy).^2);
                centroidy(temp2<0.06);
                centroidx(temp2<0.06);
                centroidx(count,1);
                if count == size(centroidx,1)
                    break;
                end
                if centroidx(count) == centroidx(count+1)
                    continue;
                end
                if size(centroidx(temp2<0.11),1) <= top && size(centroidy(temp2<0.11),1) <= top
                    if size(centroidx(temp2<0.11),1) >= 1 && size(centroidy(temp2<0.11),1) >= 1
                        if (try_theta(count,:)> angle_2) && (try_theta(count,:)< angle)
                        dist(b)  = sqrt(centroidx(count,:)^2 + centroidy(count,:)^2);
                        final_controidx(b,:) = centroidx(count,:);
                        final_controidy(b,:) = centroidy(count,:);
                        final_theta(b,:) = theta(count,:);
                        plot(centroidx(count,:),centroidy(count,:),'*b');
                        hold on
                        axis([-3 3 -3 3]);
                        b = b+1;
                        end
                    end
                end
                
            end
            [value_fin index] = min(dist);
            i = 1;
            
            
            for k=1:size(dist,2)
                [value index] = mink(dist,k);
                if value(k) > (value_fin-0.02) & value(k)< (value_fin+0.02)
                    final_value(i) = value(k)
                    px2(i) = final_controidx(index(1,k))
                    py2(i) = final_controidy(index(1,k))
                    theta2(i)= final_theta(index(1,k))
                    i=i+1;
                    
                end
                
                
            end
            pth = median(theta2)*180/pi;
            
            px = median(px2);
            py = median(py2);
            
            plot(px,py,'*g');
            hold on
            axis([-3 3 -3 3]);
            
        end
    
    
    
    
    
    
    
    
    function [rad2, po] = closestPointOnLineSegment(pi,p1,p2)
    % Given set of points and a line segment, returns the
    % closest point and square of distance to segment for
    % each point. If the closest point is an endpoint, returns
    % infinity for rad2 because such points are bad for
    % lidar matching localization.
    %
    % [rad2, po] = CLOSESTPOINTONLINESEGMENT(pi,p1,p2)
    %
    % pi - Array of points of size 2 x n.
    % p1 - Column of size 2, endpoint of segment.
    % p2 - Column of size 2, endpoint of segment.
    %
    % rad2 - Squared distance to closest point on segment.
    % po - Closest points on segment. Same size as pi.
    v1 = bsxfun(@minus,pi,p1);
    v2 = p2-p1;
    v3 = bsxfun(@minus,pi,p2);
    v1dotv2 = bsxfun(@times,v1,v2);
    v1dotv2 = sum(v1dotv2,1);
    v2dotv2 = sum(v2.*v2);
    v3dotv2 = bsxfun(@times,v3,v2);
    v3dotv2 = sum(v3dotv2,1);
    nPoints = size(pi,2);
    rad2 = zeros(1,nPoints);
    po = zeros(2,nPoints);
    % Closest is on segment
    flag1 = v1dotv2 > 0.0 & v3dotv2 < 0.0;
    if any(flag1)
        scale = v1dotv2/v2dotv2;
        temp = bsxfun(@plus,v2*scale,[p1(1) ; p1(2)]);
        po(:,flag1) = temp(:,flag1);
        dx = pi(1,flag1)-po(1,flag1);
        dy = pi(2,flag1)-po(2,flag1);
        rad2(flag1) = dx.*dx+dy.*dy;
    end
    % Closest is first endpoint
    flag2 = v1dotv2 <= 0.0;
    if any(flag2)
        temp = bsxfun(@times,ones(2,sum(flag2)),[p1(1);
            p1(2)]);
        po(:,flag2) = temp;
        rad2(flag2) = inf;
    end
    %Carnegie Mellon: Robotics Institute
    % Closest is second endpoint
    flag3 = ~flag1 & ~flag2;
    if any(flag3)
        temp = bsxfun(@times,ones(2,sum(flag3)),[p2(1);
            p2(2)]);
        po(:,flag3) = temp;
        rad2(flag3) = inf;
    end
    end
    
    
    
    
    
    
    
    end
end