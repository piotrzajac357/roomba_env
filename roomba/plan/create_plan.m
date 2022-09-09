clear all; close all;

img = imread('plan_narrow.png');
imshow(img);
img = mat2gray(img);
imwrite(img, 'plan_narrow.png');

img = imresize(img,[200 200], 'nearest');

M = img(:,:,1);
fid = fopen('plan_narrow.txt', 'w+');
for i=2:size(M, 1)
    fprintf(fid, '%d', M(i,:));
    if i~=size(M,1) 
        fprintf(fid, '\n');
    else
        fprintf(fid,'\n');
        fprintf(fid, '%d',M(1,:));
    end

end
fclose(fid);



