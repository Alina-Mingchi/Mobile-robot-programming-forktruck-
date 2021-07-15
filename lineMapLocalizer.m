 classdef lineMapLocalizer < handle
    %mapLocalizer A class to match a range scan against a map in
    % order to find the true location of the range scan relative to
    % the map.
    properties(Constant)
        maxErr = 0.1; % 5 cm
        minPts = 5; % min # of points that must match
    end
    
    properties(Access = private)
    end
    
    properties(Access = public)
        lines_p1 = [];
        lines_p2 = [];
        gain = 0.0;
        errThresh = 0.0;
        gradThresh = 0.0;
        k = 0.0;
        robot;
    end
    
    methods
        function obj = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh,k,robot)
            % create a lineMapLocalizer
            obj.lines_p1 = lines_p1;
            obj.lines_p2 = lines_p2;
            obj.gain = gain;
            obj.errThresh = errThresh;
            obj.gradThresh = gradThresh;
            obj.robot = robot;
            obj.k = k;
        end
        
        function ro2 = closestSquaredDistanceToLines(obj,pi)
            % Find the squared shortest distance from pi to any line
            % segment in the supplied list of line segments.
            % pi is an array of 2d points
            % throw away homogenous flag
            pi = pi(1:2,:);
            r2Array = zeros(size(obj.lines_p1,2),size(pi,2));
            for i = 1:size(obj.lines_p1,2)
                [r2Array(i,:) , ~] = closestPointOnLineSegment(pi,...
                    obj.lines_p1(:,i),obj.lines_p2(:,i));
            end
            ro2 = min(r2Array,[],1);
        end
        
        function ids = throwOutliers(obj,pose,ptsInModelFrame)
            % Find ids of outliers in a scan.
           
            worldPts = pose.bToA()*ptsInModelFrame;
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            ids = find(r2 > obj.maxErr*obj.maxErr);
        end
        
        function avgErr2 = fitError(obj,pose,ptsInModelFrame)
            % Find the variance  of perpendicular distances of
            % all points to all lines
            % transform the points
            ids = throwOutliers(obj,pose,ptsInModelFrame);
            global worldPts
            worldPts = pose.bToA()*ptsInModelFrame;
            worldPts(:,ids) = [];
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            r2(r2 == Inf) = [];
            err2 = sum(r2);
            num = length(r2);
            if(num >= lineMapLocalizer.minPts)
                avgErr2 = err2/num;
            else
                % not enough points to make a guess
                avgErr2 = inf;
            end
        end
        
        function [err2_Plus0,J] = getJacobian(obj,poseIn,modelPts)
            % Computes the gradient of the error function
            err2_Plus0 = fitError(obj,poseIn,modelPts); %scalar
            eps = 1e-6;
            dpx = [eps ; 0.0 ; 0.0];
            newPosex = pose(poseIn.getPoseVec+dpx);
            dpy = [0.0 ; eps ; 0.0];
            newPosey = pose(poseIn.getPoseVec+dpy);
            dpt = [0.0 ; 0.0; eps];
            newPoset = pose(poseIn.getPoseVec+dpt);
            
            newerrx = fitError(obj,newPosex,modelPts);
            newerry = fitError(obj,newPosey,modelPts);
            newerrt = fitError(obj,newPoset,modelPts);
            J = [0;0;0];
            J(1) = -(err2_Plus0 - newerrx)/eps;
            J(2) = -(err2_Plus0 - newerry)/eps;
            J(3) = -(err2_Plus0 - newerrt)/eps; 
        end
        
        function [success, outPose] = refinePose(obj,currentPose,modelPts,maxIters)
            % refine robot pose in world (inPose) based on lidar
            % registration. Terminates if maxIters iterations is
            % exceeded or if insufficient points match the lines.
            % Even if the minimum is not found, outPose will contain
            % any changes that reduced the fit error. Pose changes that 
            % increase fit error are not included and termination
            % occurs thereafter.
            
            for i  = 0 : maxIters-1
                j = i+1;
                if(size(modelPts,1) ~=3)
                   outPose = [currentPose.x currentPose.y currentPose.th];

                    success =false;
                    %break;
                    return;
                end
                [err2_Plus0,J] = getJacobian(obj,currentPose,modelPts);
                if err2_Plus0 < 0.01
                    break
                end
                currentPose = pose(currentPose.x - obj.gain * J(1),currentPose.y - obj.gain * J(2),currentPose.th - 5* J(3));
                if J(1) < 0 || J(2) < obj.gradThresh
                    break
                end

            end
            outPose = [currentPose.x currentPose.y currentPose.th];
            success = j < maxIters;    
        end
        
%         function callbackLider(obj)
%             obj.robot.startLaser();
%             obj.robot.laser.NewMessageFcn = @obj.LidarCb;
%         end
%         
%         function LidarCb(obj,~,event)
%             obj.range
%         end
        
        function Pose_est = findPose(obj,maxIters,sample,img,robot)
            figure(4)
            global encoder_pose;
            rot_matrix = [cos(pi/2), -sin(pi/2), 0;sin(pi/2), cos(pi/2), 0; 0,0,1];
            count =1;
            pause(1)
            p_est = pose(encoder_pose);
            tic;
                
            while toc < 3
                
                pause(0.1)
                
                read = obj.robot.laser.LatestMessage.Ranges;
                pause(0.5);
                r = read';
                i = 1:360;
                
                newRangeImage = removeBadPoints(img,r);
                [x y b] = irToXy(img,i,newRangeImage);
                
                x = downsample(x,10);
                y = downsample(y,10);
                xxx = find(x ~= 0);
                x = x(xxx);
                yyy = find(y ~= 0);
                y = y(yyy);
                
                modelPts = [x;y;ones(1,size(y,2))];
                
                clear j;
                [success, outPose] = refinePose(obj,p_est,modelPts,maxIters);
                
                
                
                xlabel('X [m]')
                ylabel('Y [m]')
                title('Lab 10')
                %legend('Reference','Actual')
                plot(outPose(1),outPose(2),'o')
                hold on
                
                plot(encoder_pose(1),encoder_pose(2),'*')
                new_ref = rot_matrix*[sample.refXList; sample.refYList;ones(size(sample.refYList))];
                
                plot(sample.refXList, sample.refYList)
                hold on
                plot(sample.x,sample.y)
                hold on
                
                
                p_lid = pose(outPose(1), outPose(2), outPose(3));
                if isnan(outPose(1))
                    
                    p_est = pose(encoder_pose(1),encoder_pose(2),encoder_pose(3))
                    count = count+1;
                    
                    
                else
                    p_error = [p_lid.x - p_est.x, p_lid.y - p_est.y, p_lid.th - p_est.th]
                    p_est = pose(encoder_pose(1) + obj.k*p_error(1),encoder_pose(2) + obj.k*p_error(2),encoder_pose(3) + obj.k*p_error(3))
                    count = count+1;
                    
                end
                
            end
            Pose_est = p_est;
        end
    end
 end







