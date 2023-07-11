MNI_BTBT
==

팀명 : BTBT
---

# 미니드론 자율비행을 위한 전략 및 알고리즘


## 1. 목표


![image](https://github.com/moon-123/MNI_BTBT/assets/59769304/98863e75-6402-417c-a3e5-2575331e3352)


## 2. 전략

드론이 링을 통과하기 위해선 링이 시야에 잡혀야한다.
또한 링의 구멍이 드론 시야 안에 모두 들어오는 것이 중요하다고 생각했다.
따라서 충분한 시야가 잡히지 않는다면 뒤로 이동하여 거리를 확보하기로 하였다.

드론의 최소 이동거리가 0.2임을 고려하여 뒤로 이동하여 중점을 맞추는 것이 시간측면에서 효율적이라고 생각하였다.



1단계 
- 드론 이륙, 표식 인식을 위해 상승
- 표식 인식, 중앙값 맞추기
- 표식 앞까지 전진
- 회전

2단계, 3단계
- 링이 보이지 않으면 뒤로 이동
- 원이 보이지 않으면 보이는 링 중점을 계산하여 상하좌우로 이동
  - regionprops 함수의 BoundingBox 사용
- 표식이 보이면 표식 중앙 계산 후 전진
- 만약 뒤로 이동 후 원을 본다면 원 중심 계산 후 전진

4단계
- 회전 표식을 보고 45도 회전
- 전진
- 원의 가로축 세로축 길이가 일치하도록 회전, 이동
- 표식 중앙 계산 후 전진
- 표식 픽셀값 계산 후 2m지점에서 착륙



## 3. 알고리즘
   
<img width="1270" alt="스크린샷 2023-07-10 오전 2 16 15" src="https://github.com/moon-123/MNI_BTBT/assets/59769304/63c3204b-8575-4fea-b8ec-392e09f9b24c">

<img width="1251" alt="스크린샷 2023-07-10 오전 2 19 13" src="https://github.com/moon-123/MNI_BTBT/assets/59769304/781e6dcb-1057-408d-9e1c-e0ff3125bc6b">



## 4. 코드

```matlab
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
```

```

```
