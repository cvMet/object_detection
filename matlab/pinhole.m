clear all;
%f = focal length of camera lens
f = 0.6;
zero_vector = [0 0 0]';
%(px,py) = principal point offset (coordinates)
px = 0;
py = 0;
%mx = pixel width, my = pixel height
mx = 15e-6;
my = mx;
ax = f*mx;
ay = f*my;
% principal point in terms of pixel dimensions, with coordinates x0, y0
x0 = mx * px;
y0 = my * py;
% K = Camera Calibration Matrix -> internal camera parameters
K =[ax,0,x0;
    0,ay,y0;
    0,0,1];
%P = 3×4 camera projection matrix.
P = [K, zero_vector];

%External camera paramerters
% R = Rotationsmatrix representing the orientation of the camera coordinate frame
R =[1,0,0;
    0,1,0;
    0,0,1];
% C = coordinates of the camera centre in world coordinate frame
C =[1 1 1]';
% t = translation vector
t = -R*C;

P = K * [R,t];


