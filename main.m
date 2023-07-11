clear;
[drone, cam] = Init_drone();

preview(cam);
ringDetected = false;
drone_distance = 0;

camX = 480;
camY = 200;

takeoff(drone);
moveup(drone, 'Distance', 0.3, 'Speed', 1);

% 1단계
while 1
    frame = snapshot(cam);
    disp("take a snapshot");
    pause(1);
    hsv = rgb2hsv(frame);
    h = hsv(:, :, 1);
    s = hsv(:, :, 2);
    binary_red = ((h > 0.95) & (h <= 1.0) | (h < 0.05) & (h >= 0)) & (0.6 < s) & (s < 0.97);
   
    [isXCenter, isYCenter] = make_center(binary_red, camX, camY, drone, "red");

    if ~isXCenter || ~isYCenter
        continue;
    else
        disp("arrive");
        moveforward(drone, 'distance', 2, 'speed', 1);
        % pause;
        turn(drone, deg2rad(90));
        % land(drone);
        % clear drone;
        break;
    end
end

% c = 1;
% 2단계, 3단계
for c = 1:2
    while 1
        isRingXCenter = false;
        isRingYCenter = false;
        frame = snapshot(cam);
        % disp("take a snapshot");
        pause(1);
        hsv = rgb2hsv(frame);
        h = hsv(:, :, 1);
        s = hsv(:, :, 2);
        if c == 1
            binary_red = ((h > 0.95) & (h <= 1.0) | (h < 0.05) & (h >= 0)) & (0.6 < s) & (s < 0.97);
        elseif c == 2
            binary_green = (h > 0.95) & (h <= 1.0) & (0.6 < s) & (s < 0.97); % h 범위 바꾸기
        end
    
        % if red
        if sum(binary_red, 'all') > 1500
            [isXCenter, isYCenter] = make_center(binary_red, camX, camY, drone, "red");
        
            if ~isXCenter || ~isYCenter
                continue;
          
            else 
                %disp("red_arrive");
                moveforward(drone, 'distance', 1.5 + drone_distance, 'speed', 1);
                pause(1);
                turn(drone, deg2rad(90));
                %pause(1);
                %land(drone);
                %clear drone;
                break;
            end
        % if green
        %{
        elseif sum(binary_green > 500)
    
            [isXCenter, isYCenter] = make_center(binary_green, camX, camY, drone, "green");
        
            if ~isXCenter || ~isYCenter
                continue;
          
            else 
                moveforward(drone, 'distance', 1.5 + drone_distance, 'speed', 1);
                pause;
                % turn(drone, deg2rad(90));
                % turn 각도 바꾸기
                land(drone);
                clear drone;
                break;
            end
        %}
        else
            binary_blue = (h > 0.58) & (h <= 0.65) & (0.4 < s) & (s < 0.97);
            ringDetected = find_ring(binary_blue);
            if ringDetected
                %disp("find_ring");
                mask_blue = imfill(binary_blue,'holes');
                % imshow(mask_image);
                holes = mask_blue - binary_blue;
                % imshow(holes);
                gap = abs(sum(binary_blue, 'all') - sum(mask_blue, 'all'));
                disp(gap);
                if gap < 1800
                    isHole = false;
                    %disp("not Hole");
                else
                    isHole = true;
                    %disp("is Hole");
                end
                
                if isHole
                    [isRingXCenter, isRingYCenter] = ring_center(holes, camX, camY, drone);
                else
                    % 여기

                    moveToRing(binary_blue, camX, camY, drone, drone_distance);
                end
    
                if ~isRingXCenter || ~isRingYCenter
                    continue
                else
                    moveforward(drone, 'distance', 2 + drone_distance, 'speed', 1);
                    pause(1);
                    turn(drone, rad2deg(-90));
                    %land(drone);
                    %clear drone;
                    break;
                end
    
            else
                moveback(drone, 'distance', 0.5, 'speed', 1);
                drone_distance = drone_distance + 0.5;
                continue;
            end
        end
    end
end

% 4단계
% 미완성

%{========== 함수 ==========}

function [drone, cam] = Init_drone()
    drone = ryze("Tello");
    cam = camera(drone);
end


function [flagX, flagY] = get_flags(stats)
    for i = 1:size(stats)
        if stats.MajorAxisLength(i)==max(stats.MajorAxisLength)
            maxI=i;
            break;
        end
    end
    flagX = max(stats.Centroid(maxI, 1));
    flagY = max(stats.Centroid(maxI, 2));
end

function [isXCenter, isYCenter] = make_center(binary, camX, camY, drone, color)
    isXCenter = false;
    isYCenter = false;
    %disp("function make_center");
    disp(sum(binary, 'all'));
    if color == "red"
        max = 30000;
        min = 1000;
    elseif color == "green"
        max = 30000;
        min = 1000;
    elseif color == "purple"
        max = 3000;
        min = 1000;
    end

    if sum(binary, 'all') > min && sum(binary, 'all') < max
        %disp("is red");
        stats = regionprops('table', binary, 'Centroid', 'MajorAxisLength', 'MinorAxisLength');
        [flagX, flagY] = get_flags(stats);
    
        if abs(camX - flagX) < 50
            isXCenter = true;
    
        elseif (camX < flagX)
            isXCenter = false;
            moveright(drone, 'distance', 0.2, 'Speed', 1);
    
        elseif (camX > flagX)
            isXCenter = false;
            moveleft(drone, 'distance', 0.3, 'Speed', 1);
    
        end
   
        if abs(camY - flagY) < 50
            isYCenter = true;

        elseif (camY < flagY)
            isYCenter = false;
            movedown(drone, 'distance', 0.2, 'Speed', 1);

        elseif (camY > flagY)
            isYCenter = false;
            moveup(drone, 'distance', 0.3, 'Speed', 1);
        end
    end
end

function [isXCenter, isYCenter] = ring_center(holes, camX, camY, drone)
    isXCenter = false;
    isYCenter = false;

    if sum(holes, 'all') > 1000
    %disp("is ring");
    stats = regionprops('table', holes, 'Centroid', 'MajorAxisLength', 'MinorAxisLength');
    [flagX, flagY] = get_flags(stats);

        if abs(camX - flagX) < 50
            isXCenter = true;
    
        elseif (camX < flagX)
            isXCenter = false;
            moveright(drone, 'distance', 0.2, 'Speed', 1);
    
        elseif (camX > flagX)
            isXCenter = false;
            moveleft(drone, 'distance', 0.3, 'Speed', 1);
    
        end
    
        if abs(camY - flagY) < 50
            isYCenter = true;
    
        elseif camY < flagY 
            isYCenter = false;
            movedown(drone, 'distance', 0.2, 'Speed', 1);
    
        elseif camY > flagY
            isYCenter = false;
            moveup(drone, 'distance', 0.3, 'Speed', 1);
        end
    end
end


function ringDetected = find_ring(binary_blue)
    ringDetected = false;
    if sum(binary_blue, 'all') > 1000
        ringDetected = true;
    end
end

% 0.5씩 뒤로 갈 때마다 1.4배씩 더 조절
function R = moveToRing(binary_blue, camX, camY, drone, d)
    stats = regionprops(binary_blue, 'BoundingBox');

    max = 0;
    for k = 1:length(stats)
        if max < stats(k).BoundingBox(3)
            max_idx = k;
            max = stats(k).BoundingBox(3);
        end
    end
    
    flagX = stats(max_idx).BoundingBox(1) + (stats(max_idx).BoundingBox(3) / 2);
    flagY = stats(max_idx).BoundingBox(2) + (stats(max_idx).BoundingBox(4) / 2);

    if (camX < flagX)
        moveright(drone, 'distance', 0.4 + (0.32 * d), 'Speed', 1);
    elseif (camX > flagX)
        moveleft(drone, 'distance', 0.4 + (0.32 * d), 'Speed', 1);
    end

    if camY < flagY 
        movedown(drone, 'distance', 0.4 + (0.32 * d), 'Speed', 1);
    elseif camY > flagY
        moveup(drone, 'distance', 0.4 + (0.32 * d), 'Speed', 1);
    end
end
