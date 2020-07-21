clear all; close all;
addpath('./matlab');
addpath('./external');


Zc_gt = 5.2;
%Zt    = 1.8;

% Plot word coordinate frame
Tcam   = transl(0,0,Zc_gt)*trotx(deg2rad(-90))*troty(deg2rad(25))*trotx(deg2rad(-45));
Tproj  = transl(4,0,Zc_gt)*trotx(deg2rad(-90))*troty(deg2rad(-10))*trotx(deg2rad(-45));

cam  = CentralCamera('focal', 0.015,'pixel',10e-6, ...
     'resolution',[1280 1024],'centre',[640 512],'pose',Tcam);
 
proj = CentralCamera('focal', 0.015,'pixel',10e-6, ...
     'resolution',[1280 1024],'centre',[640 512],'pose',Tproj);

world = SE3();
trplot(world,'frame','0','color','b');
xlim([-1 6]);
ylim([-1 10]);
zlim([-0.1 5.5]);

% plot camera
cam.plot_camera();
proj.plot_camera('color','y');

p0 = [0 0 0];
v1 = [1 0 0];
v2 = [0 1 0];
plane = [p0 v1 v2];
%axis([-10 10 -10 10 -10 10]);
groundplane = drawPlane3d(plane, 'facecolor','r','FaceAlpha',0.5);
%groundplane.alpha = 0.8
hold on;

%fv = stlread('./cadmodels/girl.stl');
%fv.vertices = fv.vertices * 1e-3; % go to metre scale
%patch(fv,'FaceColor',       [0.8 0.8 1.0], ...
%         'EdgeColor',       'none',        ...
%         'FaceLighting',    'gouraud',     ...
%         'AmbientStrength', 0.15);

%drawLine3d([p0 v1])
%drawLine3d([p0 v2])
set(gcf, 'renderer', 'zbuffer');
view(280,30);

sq1 = [500,500];
sq2 = [600,500];
sq3 = [600,600];
sq4 = [500,600];
projcentre = [640 512]

% ray = proj.ray(projcentre');
% rp1 = ray.P0;
% rp2 = ray.d;
% line = [rp1' rp2'];
% drawLine3d(line,'r');

ray1 = proj.ray(sq1');
rp1  = ray1.P0;
rp2  = ray1.d;
line1 = [rp1' rp2'];
drawLine3d(line1,'g');
point1 = intersectLinePlane(line1,plane);
drawPoint3d(point1(1),point1(2),point1(3),'+');

ray2 = proj.ray(sq2');
rp1  = ray2.P0;
rp2  = ray2.d;
line2 = [rp1' rp2'];
drawLine3d(line2,'g');
point2 = intersectLinePlane(line2,plane);
drawPoint3d(point2(1),point2(2),point2(3),'+');

ray3 = proj.ray(sq3');
rp1  = ray3.P0;
rp2  = ray3.d;
line3 = [rp1' rp2'];
drawLine3d(line3,'g');
point3 = intersectLinePlane(line3,plane);
drawPoint3d(point3(1),point3(2),point3(3),'+');

ray4 = proj.ray(sq4');
rp1  = ray4.P0;
rp2  = ray4.d;
line4 = [rp1' rp2'];
drawLine3d(line4,'g');
point4 = intersectLinePlane(line4,plane);
drawPoint3d(point4(1),point4(2),point4(3),'+');

% Getting the ground plane -> camera plane homography
pointsGroundPlane = [point1(1:2)' point2(1:2)' point3(1:2)' point4(1:2)']
pointsImgPlane    = cam.project([point1' point2' point3' point4'])
cHg = homography(pointsGroundPlane,pointsImgPlane);
gHc = inv(cHg);

%Getting the ground plane -> projector plane homography
pointsPrjPlane = [sq1' sq2' sq3' sq4'];
pHg = homography(pointsGroundPlane,pointsPrjPlane);
gHp = inv(pHg);

% Testing homographies on one point
testPointImgPlane  = [300,300]; % in the camera image plane
testPointProjPlane = h2e(pHg*gHc*e2h(testPointImgPlane'));

% Project both points in 3D to see if they are the same 3D point
rayTestPrj = proj.ray(testPointProjPlane);
rp1  = rayTestPrj.P0;
rp2  = rayTestPrj.d;
lineTestPrj = [rp1' rp2'];
drawLine3d(lineTestPrj,'r');
point3DTestPrj = intersectLinePlane(lineTestPrj,plane);

rayTestImg = cam.ray(testPointImgPlane');
rp1 = rayTestImg.P0;
rp2 = rayTestImg.d;
lineTestImg = [rp1' rp2'];
drawLine3d(lineTestImg,'r');
point3DTestImg = intersectLinePlane(lineTestImg,plane);
