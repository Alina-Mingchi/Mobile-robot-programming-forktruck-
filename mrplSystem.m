
classdef mrplSystem < handle
    
    properties(Access = public)
        
        curve = 0;
        refXList = zeros(1, 1);
        refYList = zeros(1, 1);
        refThList = zeros(1, 1);
        actXList = zeros(1, 1);
        actyList = zeros(1, 1);
        actThList = zeros(1, 1);

        lastRefX = 0;
        lastRefY = 0;
        lastRefTh = 0;
        robot;
        x_start;
        y_start;
        x = 0;
        y = 0;
        th = 0;
        
        
    end
    
    methods
        
        function obj = mrplSystem(robot)
            global Dx
            global Dy
            obj.robot = robot;
            obj.x_start = Dx;
            obj.y_start = Dy;
            
        end
        
        function obj = setTrajectory(obj,curve)
            obj.curve = curve;
        end
        
        
        function [refXList, refYList, refThList] = referenceTrajectory(obj)
            tic;
            refXList = zeros(1, 1);
            refYList = zeros(1, 1);
            refThList = zeros(1, 1);
            while toc < obj.curve.getTrajectoryDuration()
                t = toc;
                refPose = obj.curve.getPoseAtTime(t);
%                 obj.refXList(end+1) = refPose(1) + obj.lastRefX;
%                 obj.refYList(end+1) = refPose(2) + obj.lastRefY;
%                 obj.refThList(end+1) = refPose(3) + obj.lastRefTh;
                
            end
            obj.lastStateOfReference();
            
        end
        
        
        function obj = lastStateOfReference(obj)
            obj.lastRefX =  obj.refXList(end);
            obj.lastRefY =  obj.refYList(end);
            obj.lastRefTh = obj.refThList(end);
            
        end
        
        function backup_turn(obj,robot)
            
            obj.robot.sendVelocity(-0.1,-0.1)
            pause(1)
            obj.robot.stop()
            obj.robot.sendVelocity(0.045,-0.05)
            pause(2)
        end
        
        
        
        
        function [x, y, theta] = estimateTrajectory(obj)
            
            global x_start
            global y_start
            global Dx
            global Dy
            global newTheta
            global th_start
            x = Dx;
            y = Dy;
            theta = newTheta;
        end
        
        
        
        function executeTrajectoryToRelativePose(obj,robot)
            i=1;
            tic;
            global Dx
            global x_start
            global a
            
 
            while toc< obj.curve.getTrajectoryDuration()
                
                t = toc;
                
                V = obj.curve.getVAtTime(t);
                w = obj.curve.getwAtTime(t);
                vl = V - (0.09/2)*w;
                vr = V + (0.09/2)*w;
                
                refPose = obj.curve.getPoseAtTime(t);
                
%                 obj.refXList(end+1) = refPose(1) + obj.lastRefX;
%                 obj.refYList(end+1) = refPose(2) + obj.lastRefY;
%                 obj.refThList(end+1) = refPose(3) + obj.lastRefTh;
                T = [cos(obj.lastRefTh),-sin(obj.lastRefTh),obj.lastRefX;...
                    sin(obj.lastRefTh),cos(obj.lastRefTh),obj.lastRefY;...
                    0,0,1];
                pose_world = T*[refPose(1);refPose(2);1];
                obj.refXList(end+1) = pose_world(1);
                obj.refYList(end+1) = pose_world(2);
                obj.refThList(end+1) = refPose(3);
                
                [obj.x(a), obj.y(a), obj.th(a)] = estimateTrajectory(obj);
                
                obj.robot.sendVelocity(vl,vr);
                pause(0.05)
                
                a=a+1;
            end
            robot.stop()
            obj.lastStateOfReference();
            
            pause(0.5)
            
        end
    end
end
