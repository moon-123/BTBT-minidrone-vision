clear;
[drone, cam] = Init_drone();

takeoff(drone);
pause(1);
moveup(drone, 'Distance', 0.3, 'Speed', 1, 'WaitUntilDone', false);
pause(1);


ringDetected = false;
isRingXCenter = false;
isRingYCenter = false;
isHorizon = false;
no_more_blue = false;
binary_red = 0;
binary_green = 0;
binary_blue = 0;

camX = 480;
camY = 210;

for c = 1:3
    isRed = false;
    isGreen = false;
    while 1
        frame = snapshot(cam);
        pause(1);
        [R, G, B] = makeRealRGB(frame);
        [h, s] = makeHSV(frame);
        pause(1);

        if c == 1 || c == 2
            binary_red = ((h > 0.97) | (h < 0.02)) & (0.7 < s) & (s < 0.97) & R;
        elseif c == 3
            binary_green = (h > 0.25) & (h <= 0.4) & (0.1 < s) & (s < 0.6) & G;
        end
        disp(sum(binary_red, 'all'));

        % if red
        if sum(binary_red, 'all') > 300
            disp("==find_red==");
            isRed = true;
            [isXCenter, isYCenter] = make_center(binary_red, camX, camY, drone);
        
            if ~isXCenter || ~isYCenter
                continue;
          
            else 
                disp("red_arrive");
                moveforward(drone, 'distance', 2, 'speed', 1, 'WaitUntilDone', false);
                pause(2);
                turn(drone, deg2rad(90));
                pause(2);
                break;
            end

        % if green
        elseif sum(binary_green, 'all') > 300
            disp("==find_green==");
            isGreen = true;
            [isXCenter, isYCenter] = make_center(binary_green, camX, camY, drone);
        
            if ~isXCenter || ~isYCenter
                continue;
          
            else 
                disp("green_arrive");
                moveforward(drone, 'distance', 2, 'speed', 1, 'WaitUntilDone', false);
                pause(2);
                turn(drone, deg2rad(45));
                break;
            end
        
        elseif ~isRed && ~isGreen
            disp("==find_blue==");
            binary_blue = (h > 0.52) & (h <= 0.68) & (0.4 < s) & (s < 0.97) & B;
            moveToRing(binary_blue, camX, camY, drone);
        end
    end
end

% 4단계
disp("level4");
while 1
    frame = snapshot(cam);
    pause(1);
    [R, G, B] = makeRealRGB(frame);
    [h, s] = makeHSV(frame);
    purple = makePurple(frame);

    binary_blue = (h > 0.52) & (h <= 0.68) & (0.4 < s) & (s < 0.97) & B;
    binary_purple = (h > 0.7) & (h <= 0.9) & (0.1 < s) & (s < 0.6) & purple;

    if ~no_more_blue
        [isXCenter, isYCenter] = make_center(binary_blue, camX, camY, drone);
    else
        [isXCenter, isYCenter] = make_center(binary_purple, camX, camY, drone);
    end
    
    if (~isXCenter || ~isYCenter)
        continue;
    else
        disp("blue ok");
        if ~isHorizon
            disp("make Horizon");
            isHorizon = makeHorizon(binary_blue, drone);
        else
            if sum(binary_purple, 'all') < 2100
                disp(sum(binary_purple, 'all'));
                moveforward(drone, 'distance', 0.2, 'WaitUntilDone', false);
                pause(1);
                no_more_blue = true;
            else
                land(drone);
                clear drone;
                break;
            end
        end
    end
end

%{========== 함수 ==========}

function [drone, cam] = Init_drone()
    drone = ryze("Tello");
    pause(1);
    cam = camera(drone);
    pause(1);
end

function [realR, realG, realB] = makeRealRGB(frame)
    R = frame(:, :, 1);
    G = frame(:, :, 2);
    B = frame(:, :, 3);    
    realR = (R - G/2 - B/2) > 100;
    realG = (G - R/2 - B/2) > 20;
    realB = (B - R/2 - G/2) > 20;
end

function [h, s] = makeHSV(frame)
    hsv = rgb2hsv(frame);
    h = hsv(:, :, 1);
    s = hsv(:, :, 2);
end

function [purple] = makePurple(frame)
    R = frame(:, :, 1);
    G = frame(:, :, 2);
    B = frame(:, :, 3);
    purple_R = R > 80 & R < 140;
    purple_G = G > 60 & G < 100;
    purple_B = B > 90 & B < 140;
    purple = purple_R & purple_G & purple_B;
end

function [flagX, flagY] = get_flags(stats)
    max = 0;
    
    for k = 1:length(stats)
        if max < stats(k).BoundingBox(3)
            max_idx = k;
            max = stats(k).BoundingBox(3);
        end
    end

    flagX = stats(max_idx).BoundingBox(1) + (stats(max_idx).BoundingBox(3) / 2);
    flagY = stats(max_idx).BoundingBox(2) + (stats(max_idx).BoundingBox(4) / 2);
end

% 표식 중점 찾아가는 함수
function [isXCenter, isYCenter] = make_center(binary, camX, camY, drone)
    isXCenter = false;
    isYCenter = false;
    %disp("function make_center");

    stats = regionprops(binary, 'BoundingBox');
    [flagX, flagY] = get_flags(stats);

    if abs(camX - flagX) < 50
        isXCenter = true;

    elseif (camX < flagX)
        isXCenter = false;
        moveright(drone, 'distance', 0.2, 'Speed', 1, 'WaitUntilDone', false);
        pause(2);

    elseif (camX > flagX)
        isXCenter = false;
        moveleft(drone, 'distance', 0.3, 'Speed', 1, 'WaitUntilDone', false);
        pause(2);

    end

    if abs(camY - flagY) < 50
        isYCenter = true;

    elseif (camY < flagY)
        isYCenter = false;
        movedown(drone, 'distance', 0.2, 'Speed', 1, 'WaitUntilDone', false);
        pause(2);

    elseif (camY > flagY)
        isYCenter = false;
        moveup(drone, 'distance', 0.3, 'Speed', 1, 'WaitUntilDone', false);
        pause(2);
    end
    
end


function [yaw_y_left, yaw_y_right] = getHorizon(image)
    box = regionprops(image, 'Image');
    m = 0;
    for k = 1:length(box)
        if m < sum(box(k).Image, 'all')
            m_idx = k;
            m = sum(box(k).Image, 'all');
        end
    end
    box_image = box(m_idx).Image;
    [height, width] = size(box_image);

    for j= 1:width/2
        yaw_y_left= height-j+1;
        yaw_x_left= 1 ;
        while(1)
          find_yaw_left=0;

            if box_image(yaw_y_left,yaw_x_left) ==1
                find_yaw_left=1;
                break;
            elseif yaw_y_left ==height && yaw_x_left==j
                break;   
            end

        yaw_y_left=yaw_y_left+1;
        yaw_x_left=yaw_x_left+1;

        end

        if find_yaw_left==1
            break;
        end
    end
    
    for j= 1:width/2
        yaw_y_right= height-j+1;
        yaw_x_right= width ;

        while 1
        find_yaw_right=0;

            if box_image(yaw_y_right,yaw_x_right) ==1
                find_yaw_right=1;
                break;
            elseif yaw_y_right ==height && yaw_x_right==width-j+1
                break;   
            end

        yaw_y_right=yaw_y_right+1;
        yaw_x_right=yaw_x_right-1;

        end

        if find_yaw_right==1
            break;
        end
    end
end

function isHorizon = makeHorizon(image, drone)
    [y_left, y_right] = getHorizon(image);
    if abs(y_left - y_right) < 20
        isHorizon = true;
    elseif y_left < y_right
        turn(drone, deg2rad(7));
        isHorizon = false;
    elseif y_left > y_right
        turn(drone, deg2rad(-5));
        isHorizon = false;
    end
end
