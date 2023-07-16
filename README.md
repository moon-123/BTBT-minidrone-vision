MNI_BTBT
==

팀명 : BTBT  
팀원 : 양문기
---

대회 결과
<br> 수상: 은상 </br>

# 미니드론 자율비행을 위한 전략 및 알고리즘


# 1. 목표

Ryze사의 Tello drone을 사용하여
객체를 인식하고 분석하여 자율비행을 구현한다.

<br> 4가지 단계로 이뤄진 경기장을 자율비행하여 착륙지점에 정확하게 착륙하는 것이 목표 </br>

#### 경기장 스펙
<img width="400" alt="image" src="https://github.com/moon-123/MNI_BTBT/assets/59769304/98863e75-6402-417c-a3e5-2575331e3352">

각각의 링의 위치는 좌우 200m, 상하 50cm ~ 150cm 까지 변동될 수 있고 4단계링은 추가로 각도가 변동될 수 있다.

# 2. 전략

실제 링 스펙과 비슷한 스펙으로 간이 트랙을 제작함  
<img width="400" alt="KakaoTalk_Photo_2023-07-12-23-45-38" src="https://github.com/moon-123/MNI_BTBT/assets/59769304/89930c40-487b-401b-8ef7-fd8cc44763ab">  


전체동작  
- 드론 카메라를 통해 영상처리, 모터 제어를 통한 이동의 반복으로 자율비행을 하게된다.  

영상처리  
- 정확한 색상을 인식하기 위해 RGB, HSV 값을 모두 사용함.
- 크게 링만 보일 때, 표식이 보일 때를 나눔. 표식과 링이 같이 보여도 표식만 보인것으로 판단
- 만약 표식을 놓치게 되면 다시 링만 보일때의 동작으로 돌아감.

제어
- 반복적인 이동보단 한 번의 정확한 이동이 시간효율에 좋다고 판단.


리허설 후 표식과 링 색상을 정확하게 잡는것이 중요하다고 판단함


드론이 링을 통과하기 위해선 링이 시야에 잡혀야한다.
또한 링의 구멍이 드론 시야 안에 모두 들어오는 것이 중요하다고 생각했다.
따라서 충분한 시야가 잡히지 않는다면 뒤로 이동하여 거리를 확보하기로 하였다.

드론의 최소 이동거리가 0.2임을 고려하여 뒤로 이동하여 중점을 맞추는 것이 시간측면에서 효율적이라고 생각하였다.



#### 1단계 
- 드론 이륙
- 링 인식 후 중점방향으로 이동
(regionprops 함수의 BoundingBox 사용)
- 표식 인식, 중앙값 맞추기
- 표식 앞까지 전진
- 회전

#### 2단계, 3단계
- 표식이 보이면
  - 표식 인식, 중앙값 맞추기
  - 표식 앞까지 전진
  - 회전
 
- 표식이 보이지 않으면
  - 링 인식 후 중점방향으로 이동
  (regionprops 함수의 BoundingBox 사용)
  - 표식 인식, 중앙값 맞추기
  - 표식 앞까지 전진
  - 회전

#### 4단계
- 회전 표식을 보고 45도 회전
- 전진
- 원의 가로축 세로축 길이가 일치하도록 회전, 이동
- 표식 중앙 계산 후 전진
- 표식 픽셀값 계산 후 2m지점에서 착륙


# 3. 알고리즘

<img width="1270" alt="스크린샷 2023-07-10 오전 2 16 15" src="https://github.com/moon-123/MNI_BTBT/assets/59769304/63c3204b-8575-4fea-b8ec-392e09f9b24c">

<img width="528" alt="스크린샷 2023-07-12 오후 11 16 10" src="https://github.com/moon-123/MNI_BTBT/assets/59769304/04baec80-e76f-43b5-9913-da00708d3f3d">


# 4. 코드

- 1, 2, 3단계의 경우 동작이 비슷하기 때문에 for문을 사용함
- 반복되는 부분의 경우 함수로 만들어 코드 가독성이 좋게 함
(코드블럭 위가 해당 블럭 설명입니다)

### Code
- 드론 초기화와 변수 선언
- 이륙 후 초기 위치 조정
```matlab
clear;
[drone, cam] = Init_drone();

preview(cam);
ringDetected = false;
isRingXCenter = false;
isRingYCenter = false;
drone_distance = 0;

camX = 480;
camY = 200;

takeoff(drone);
moveup(drone, 'Distance', 0.3, 'Speed', 1);
```

- 기본 동작
- c 값에 따라 찾아야하는 표식을 다르게 설정
- 이미지 전처리 과정
```matlab
for c = 1:3
    while 1
        frame = snapshot(cam);
        % disp("take a snapshot");
        pause(1);
        hsv = rgb2hsv(frame);
        h = hsv(:, :, 1);
        s = hsv(:, :, 2);
        if c == 1 || c == 2
            binary_red = ((h > 0.95) & (h <= 1.0) | (h < 0.05) & (h >= 0)) & (0.6 < s) & (s < 0.97);
        elseif c == 3
            binary_green = (h > 0.95) & (h <= 1.0) & (0.6 < s) & (s < 0.97); % h 범위 바꾸기
        end
        ...
```

- 표식이 red일 때 표식이 보이면 실행되는 부분
- 표식 중앙이 맞으면 전진 후 회전
- 동작이 끝났으므로 while문 break
```matlab
        % if red, c == 1, 2
        if sum(binary_red, 'all') > 1500
            [isXCenter, isYCenter] = make_center(binary_red, camX, camY, drone, "red");
        
            if ~isXCenter || ~isYCenter
                continue;
          
            else 
                %disp("red_arrive");
                moveforward(drone, 'distance', 1.5 + drone_distance, 'speed', 1);
                pause(1);
                turn(drone, deg2rad(90));
                break;
            end
```

- 표식이 green일 때 표식이 보이면 실행되는 부분
- 표식 중앙이 맞으면 전진 후 회전
- 동작이 끝났으므로 while문 break
```matlab
        % if green, c == 3
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
```

- 표식이 보이지 않는다면 링을 인식
- 링 중점 계산하고 이동하는 부분
- 원이 인식되는지 확인하기 위해 gap 변수 설정
  - gap이 일정값 이상이면 원이 인식된 것
```matlab
        ...
        else
            binary_blue = (h > 0.58) & (h <= 0.65) & (0.4 < s) & (s < 0.97);
            ringDetected = find_ring(binary_blue);
            if ringDetected
                disp("find_ring");
                mask_blue = imfill(binary_blue,'holes');
                holes = mask_blue - binary_blue;
                gap = abs(sum(binary_blue, 'all') - sum(mask_blue, 'all'));
                disp(gap);
                if gap < 1800
                    isHole = false;
                    disp("not Hole");
                else
                    isHole = true;
                    disp("is Hole");
                end
                ...
```

- 만약 원이 인식되면 실행되는 부분
- ring_center는 원의 중점을 찾아 드론을 제어하는 함수
```matlab
                ...
                if isHole
                    [isRingXCenter, isRingYCenter] = ring_center(holes, camX, camY, drone);
                ...
```

- 만약 원이 인식되지 않는다면 실행되는 부분
- moveToRing은 보이는 링의 중점을 계산하여 드론을 링 중앙쪽으로 제어하는 함수
```matlab
                ...
                else
                    moveToRing(binary_blue, camX, camY, drone, drone_distance);
                end
                ...
```

- 원 중점이 맞으면 드론을 전진 후 회전
- 동작이 끝나면 while문 break
```matlab
                ...
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
                ...
```

- 링과 표식도 보이지 않는다면 뒤로 이동
- 이동한 거리만큼 더 전진해야 하기 때문에 drone_distance로 이동한 거리 저장
```matlab
            ...
            else
                moveback(drone, 'distance', 0.5, 'speed', 1);
                drone_distance = drone_distance + 0.5;
                continue;
            end
        end
    end
end
```

### Functions

- 드론 초기화
```matlab
function [drone, cam] = Init_drone()
    drone = ryze("Tello");
    cam = camera(drone);
end
```

- 객체의 중점을 계산하는 함수
```matlab
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
```
- 표식의 중점을 찾아 이동하는 함수
```matlab
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
```

- 원의 중점을 찾아 이동하는 함수
```matlab
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
```

- 링을 찾으면 TRUE를 반환하는 함수
```matlab
function ringDetected = find_ring(binary_blue)
    ringDetected = false;
    if sum(binary_blue, 'all') > 1000
        ringDetected = true;
    end
end
```

- 링의 중점을 찾아 이동하는 함수
```matlab
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
```

