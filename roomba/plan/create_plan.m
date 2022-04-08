clear all; close all;

img = imread('plan6.png');
imshow(img);
img = mat2gray(img);
img = imresize(img,[200 200], 'nearest');

M = img(:,:,1);
fid = fopen('plan_binary1.txt', 'w+');
for i=1:size(M, 1)
    fprintf(fid, '%d', M(i,:));
    fprintf(fid, '\n');
end
fclose(fid);