clear;
image = imread("img/문제5.png");
% imshow(image);
image_hsv = rgb2hsv(image);

image_hue = image_hsv(:, :, 1);
image_sat = image_hsv(:, :, 2);
% image_val = image_hsv(:, :, 3);

image_hsv_green = double(zeros(size(image_hue))); 

for i = 1: size(image_hsv_green, 1)
    for j = 1:size(image_hsv_green, 2)
        if (image_hue(i, j) > 0.285 && image_hue(i, j) < 0.385) && (image_sat(i, j) < 0.97) && (image_sat(i,j) > 0.4) 
            image_hsv_green(i, j) = 1;
        end
    end
end

% imshow(image_hsv_green);  

mask_image = imfill(image_hsv_green,'holes');
% imshow(mask_image);
holes = mask_image - image_hsv_green;
% imshow(holes);

stats = regionprops('table',holes,'Centroid');

centerX = stats.Centroid(1);
centerY = stats.Centroid(2);
disp("centerX : " + centerX);
disp("centerY : " + centerY);
% rectangle('Position', [centerX, centerY, 1, 1], 'FaceColor', "r");

