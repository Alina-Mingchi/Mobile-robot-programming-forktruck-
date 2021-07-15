
close all
%clearvars -except robot
clear all
robot = raspbot();
robot.startLaser();
robot.encoders.NewMessageFcn=@encoderEventListener;

pause(1);

global keypressDataReady;
global keypressKey;
global centroidx;
global centroidy;
global worldPts;
global encoder_pose;
global Dx Dy newTheta;
global a
a = 1
Dx = 0.6096;
Dy = 0.6096;
newTheta = pi/2;
pause(0.5)


gain = 0.8;
errThresh = 0.01;
gradThresh = 0.0005;
maxIters = 20;
vGain = 2;
p_est = pose(encoder_pose);
k = 0.25;
count = 1;
% Set up lines
p1 = [0;0];
p2 = [2;0];
p3 = [0;2];

lines_p1 = [p1 p3];
lines_p2 = [p2 p1];
figure(1)


rot_matrix = [cos(pi/2), sin(pi/2), 0;-sin(pi/2), cos(pi/2), 0; 0,0,1];


SP1 = cubicSpiralTrajectory.planTrajectory(0.3048,0.3048,0,1);
SP1.planVelocities(.2);



sample = mrplSystem(robot);

setTrajectory(sample, SP1);
sample.executeTrajectoryToRelativePose(robot);
tic;
p_est = pose(encoder_pose);

while toc < 6
 
    %keydrive.drive(robot,vGain)
    pause(0.1)
    
    read = robot.laser.LatestMessage.Ranges;
    pause(0.5);
    r = read';
    i = 1:360;
    
    img = rangeImage();
    newRangeImage = removeBadPoints(img,r);
    [x y b] = irToXy(img,i,newRangeImage);
    
    x = downsample(x,10);
    y = downsample(y,10);
    xxx = find(x ~= 0);
    x = x(xxx);
    yyy = find(y ~= 0);
    y = y(yyy);
    modelPts = [x;y;ones(1,size(y,2))];
 
    test0 = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh);
    clear j;
    [success, outPose] = refinePose(test0,p_est,modelPts,maxIters);
    
    plot(lines_p1,lines_p2,'r')
       % axis([2 -2 2 -2])

    hold on
%     plot(worldPts(1,:),worldPts(2,:),'*')
%         hold on

    xlabel('X [m]')
    ylabel('Y [m]')
    title('Lab 10')
    legend('Reference','Actual')
    plot(outPose(1),outPose(2),'o')
    hold on

    plot(encoder_pose(1),encoder_pose(2),'*')
    new_ref = rot_matrix*[sample.refXList; sample.refYList;ones(size(sample.refYList))]
    plot(sample.refXList, sample.refYList)
    hold on
    plot(sample.x,sample.y)
    hold on
    sample.x
    p_lid = pose(outPose(1), outPose(2), outPose(3));
    p_error = [p_lid.x - p_est.x, p_lid.y - p_est.y, p_lid.th - p_est.th]
    p_est = pose(encoder_pose(1) + k*p_error(1),encoder_pose(2) + k*p_error(2),encoder_pose(3) + k*p_error(3))
    count = count+1;
    
    

end


new_pose = inv([cos(p_est.th), -sin(p_est.th), p_est.x;sin(p_est.th), cos(p_est.th), p_est.y;...
    0,0,1])* [0.9144; 0.3048; 1];



SP2 = cubicSpiralTrajectory.planTrajectory(new_pose(1),new_pose(2),-p_est.th,1);
SP2.planVelocities(.2);



setTrajectory(sample, SP2);
sample.executeTrajectoryToRelativePose(robot);
tic;

p_est = pose(encoder_pose);

while toc < 6
 
    %keydrive.drive(robot,vGain)
    pause(0.1)
    
    read = robot.laser.LatestMessage.Ranges;
    pause(0.5);
    r = read';
    i = 1:360;
    
    img = rangeImage();
    newRangeImage = removeBadPoints(img,r);
    [x y b] = irToXy(img,i,newRangeImage);
    
    x = downsample(x,10);
    y = downsample(y,10);
    xxx = find(x ~= 0);
    x = x(xxx);
    yyy = find(y ~= 0);
    y = y(yyy);
    modelPts = [x;y;ones(1,size(y,2))];
 
    test0 = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh);
    clear j;
    [success, outPose] = refinePose(test0,p_est,modelPts,maxIters);
    
    plot(lines_p1,lines_p2,'r')
       % axis([2 -2 2 -2])

    hold on
%     plot(worldPts(1,:),worldPts(2,:),'*')
%         hold on

    xlabel('X [m]')
    ylabel('Y [m]')
    title('Lab 10')
    legend('Reference','Actual')
    plot(outPose(1),outPose(2),'o')
    hold on

    plot(encoder_pose(1),encoder_pose(2),'*')
    plot(sample.refXList, sample.refYList)
    hold on
    plot(sample.x,sample.y)
    
    hold on
    p_lid = pose(outPose(1), outPose(2), outPose(3));
    p_error = [p_lid.x - p_est.x, p_lid.y - p_est.y, p_lid.th - p_est.th]
    p_est = pose(encoder_pose(1) + k*p_error(1),encoder_pose(2) + k*p_error(2),encoder_pose(3) + k*p_error(3));
    count = count+1;
    
    

end




new_pose = inv([cos(p_est.th), -sin(p_est.th), p_est.x;sin(p_est.th), cos(p_est.th), p_est.y;...
    0,0,1])* [0.6096; 0.6096; 1];



SP3 = cubicSpiralTrajectory.planTrajectory(new_pose(1),new_pose(2),(pi/2)-p_est.th,1);
SP3.planVelocities(.2);



setTrajectory(sample, SP3);
sample.executeTrajectoryToRelativePose(robot);
tic;
p_est = pose(encoder_pose);
tic;
while toc < 6
 
    %keydrive.drive(robot,vGain)
    pause(0.1)
    
    read = robot.laser.LatestMessage.Ranges;
    pause(0.5);
    r = read';
    i = 1:360;
    
    img = rangeImage();
    newRangeImage = removeBadPoints(img,r);
    [x y b] = irToXy(img,i,newRangeImage);
    
    x = downsample(x,10);
    y = downsample(y,10);
    xxx = find(x ~= 0);
    x = x(xxx);
    yyy = find(y ~= 0);
    y = y(yyy);
    modelPts = [x;y;ones(1,size(y,2))];
 
    test0 = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh);
    clear j;
    [success, outPose] = refinePose(test0,p_est,modelPts,maxIters);
    
    plot(lines_p1,lines_p2,'r')
       % axis([2 -2 2 -2])

    hold on
%     plot(worldPts(1,:),worldPts(2,:),'*')
%         hold on

    xlabel('X [m]')
    ylabel('Y [m]')
    title('Lab 10')
    legend('Reference','Actual')
    plot(outPose(1),outPose(2),'o')
    hold on

    plot(encoder_pose(1),encoder_pose(2),'*')
    plot(sample.refXList, sample.refYList)
    hold on
    plot(sample.x,sample.y)
    
    hold on
    p_lid = pose(outPose(1), outPose(2), outPose(3));
    p_error = [p_lid.x - p_est.x, p_lid.y - p_est.y, p_lid.th - p_est.th]
    p_est = pose(encoder_pose(1) + k*p_error(1),encoder_pose(2) + k*p_error(2),encoder_pose(3) + k*p_error(3));
    count = count+1;
    
    

end






