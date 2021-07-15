function encoderEventListener(handle,event)
%% Defining Variables
global currentTime
global prevTime

global Delta
global newDx newDy
global oldDx oldDy
global sumdisp
global encoder_pose
global vleft vright V_robot

global EncX EncY
global EncX_prev EncY_prev

global oldTheta newTheta
global Omega_gl

global Dx Dy
global k_s

global dist


tstamp = double(event.Header.Stamp.Sec) + double(event.Header.Stamp.Nsec)/1e9;

if isempty(EncX)
    EncX = event.Vector.X;
    EncY = event.Vector.Y;
    Dy = 0;
    Dx = 0;
    newTheta = 0; 
    Omega_gl = 0;
    Delta = 0;
    sumdisp = 0;
    currentTime = tstamp;
    return
end


   
    


%% Calculating Theta, Velocity and X/Y

oldTheta = newTheta;
EncX_prev = EncX;
EncY_prev = EncY;
prevTime = currentTime;

currentTime = tstamp;

EncX = event.Vector.X;
EncY = event.Vector.Y;


timestep = currentTime - prevTime;

vleft = ((EncX-EncX_prev)/(currentTime - prevTime));
    
vright = ((EncY-EncY_prev)/(currentTime - prevTime));

Omega_gl = (vright-vleft)/.09;

newTheta = (Omega_gl * timestep)/2 + oldTheta;

V_robot = (vleft + vright)/2;

Delta = V_robot*timestep; 
sumdisp = sumdisp + Delta;

Dx = Delta*cos(newTheta) + Dx;
Dy = Delta*sin(newTheta) + Dy;

newTheta = (Omega_gl * timestep)/2 + newTheta;

%encoder_pose = pose(Dx, Dy, newTheta);
encoder_pose = [Dx, Dy, newTheta];

end
