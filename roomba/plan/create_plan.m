clear all; close all;

img = imread('plan_clover.png');
imshow(img);
img = mat2gray(img);
img = imresize(img,[200 200], 'nearest');

M = img(:,:,1);
fid = fopen('plan_clover.txt', 'w+');
for i=1:size(M, 1)
    fprintf(fid, '%d', M(i,:));
    if i~=size(M,1) 
        fprintf(fid, '\n');
    end
end
fclose(fid);