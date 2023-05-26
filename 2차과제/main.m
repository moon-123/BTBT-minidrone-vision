% initializing
close all
clear

% setting
droneObj = ryze("Tello");
cam = camera(droneObj);
preview(cam);
% location_vector = {false, 0, 0};
% height, x, y

takeoff(droneObj);
% location_vector(1) = true; 
% pause(2);

moveleft(droneObj, 'Distance', 2, 'WaitUntilDone', true);
pause(3);

turn(droneObj, deg2rad(30));
% pause(2);

moveforward(droneObj, 'Distance', 1, 'WaitUntilDone', true);
pause(3);

turn(droneObj, deg2rad(60));
% pause(2);

moveforward(droneObj, 'Distance', 1, 'WaitUntilDone', true);

image = snapshot(cam);
imshow(image);
pause(3);

turn(droneObj, deg2rad(-30));
% pause(2);

moveright(droneObj, 'Distance', 1, 'WaitUntilDone', true);
pause(3);

turn(droneObj, deg2rad(-60));
% pause(2);

land(droneObj);
clear droneObj;

% 만약 드론객체가 선언되어있지 않으면 선언
