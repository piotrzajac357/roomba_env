clear all;

filename = 'C:\Users\zajac\Desktop\pm\roomba_env\roomba\log\map.txt';
startRow = 2;
formatSpec = '%s%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', '', 'WhiteSpace', '', 'TextType', 'string', 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray{1} = strtrim(dataArray{1});
fclose(fileID);
map = [dataArray{1:end-1}];
clearvars filename startRow formatSpec fileID dataArray ans;

img = zeros(39,40);
for i=1:1:39
    str = char(map(i,1))-48;
    for j=1:1:40
        img(i,j) = str(j)/4;
    end
end

figure();
subplot(121);
imshow(imread("plan5.png"));
title('plan mieszkania')
subplot(122);
imshow(img,'InitialMagnification',1200);
title('discovery map');

%%
%filename = 'C:\Users\zajac\Desktop\pm\roomba_env\roomba\log\map_parent.txt';
filename = 'C:\Users\zajac\Desktop\pm\some_backup\map_parent.txt';
startRow = 2;
formatSpec = '%s%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', '', 'WhiteSpace', '', 'TextType', 'string', 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray{1} = strtrim(dataArray{1});
fclose(fileID);
map = [dataArray{1:end-1}];
clearvars filename startRow formatSpec fileID dataArray ans;

img = zeros(39,40);
for i=1:1:39
    str = char(map(i,1))-48;
    for j=1:1:40
        img(i,j) = str(j)/4;
    end
end
figure(); 
imshow(img,'InitialMagnification',1200);
title('parent cell map with spanning tree');
%%figure()
for i=1:1:38
    for j=1:1:39
        if (img(i,j) == img(i,j+1) && (img(i,j) == 0.5 || img(i,j) == 1))
            % in new plot
            %line([j j+1],[39-i 39-i]);
            line([j j+1], [i i]);
        end
        if (img(i,j) == img(i+1,j) && (img(i,j) == 0.25 || img(i,j) == 0.75))
            % in new plot
            %line([j j],[39-i 39-i+1]);
            line([j j], [i i+1]);
        end
        if ((img(i,j) == 0.5 && img(i+1,j) == 0.75) ||...
            (img(i,j) == 0.25 && img(i+1,j) == 1) || ...
            (img(i,j) == 0.25 && img(i+1,j) == 0.5) || ...
            (img(i,j) == 1 && img(i+1,j) == 0.75))
            % in new plot
            %line([j j],[39-i 39-i+1]);
            line([j j], [i i+1]);
        end
        if ((img(i,j) == 0.75 && img(i,j+1) == 1) || ...
            (img(i,j) == 0.5 && img(i,j+1) == 0.75) || ...
            (img(i,j) == 0.5 && img(i,j+1) == 0.25) || ...
            (img(i,j) == 0.25 && img(i,j+1) == 1))
            % in new plot
            %line([j j],[39-i 39-i+1]);
            line([j j+1], [i i]);
        end
    end
end

for i=1:1:38
    for j=1:1:39
        if img(i,j) == 0.5 && img(i,j+1) == 0.5
            hold on;
            quiver(j+1.25,i-0.25,-1,0,0,'LineWidth',1,'Color',[1 0 0],'MaxHeadSize',3);
        end
        if img(i,j+1) == 1 && img(i,j) == 1
            hold on;
            quiver(j-0.25,i+0.25,1,0,0,'LineWidth',1,'Color',[1 0 0],'MaxHeadSize',3);
        end
        if img(i,j) == 0.75 && img(i+1,j) == 0.75
            hold on;
            quiver(j-0.25,i-0.25,0,1,0,'LineWidth',1,'Color',[1 0 0],'MaxHeadSize',3);
        end

        if img(i+1,j) == 0.25 && img(i,j) == 0.25
            hold on;
            quiver(j+0.25,i+1.25,0,-1,0,'LineWidth',1,'Color',[1 0 0],'MaxHeadSize',3);
        end
        if img(i,j+1) == 0.75 && img(i,j) == 0.5
            hold on;
            quiver(j+0.75,i-0.25,-1,0,0,'LineWidth',1,'Color',[1 0 0],'MaxHeadSize',3);
        end

    end
end
%%
filename = 'C:\Users\zajac\Desktop\pm\roomba_env\roomba\log\map_cov.txt';
startRow = 2;
formatSpec = '%s%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', '', 'WhiteSpace', '', 'TextType', 'string', 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray{1} = strtrim(dataArray{1});
fclose(fileID);
map = [dataArray{1:end-1}];
clearvars filename startRow formatSpec fileID dataArray ans;

img = zeros(799,799);
for i=1:1:799
    str = char(map(i,1))-48;
    for j=1:1:800
        img(i,j) = str(j)/2;
    end
end
imshow(img);

%%
filename = 'C:\Users\zajac\Desktop\pm\roomba_env\roomba\log\log_cov.txt';
opts = delimitedTextImportOptions("NumVariables", 1);
opts.DataLines = [1, Inf];
opts.Delimiter = ",";
opts.VariableNames = "cov_vector";
opts.VariableTypes = "double";
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
cov_vector = readtable("C:\Users\zajac\Desktop\pm\roomba_env\roomba\log\log_cov.txt", opts);
cov_vector = table2array(cov_vector);
clear opts

time_stamp = 1/60:1/60:size(cov_vector,1)/60;

plot(time_stamp,cov_vector.*100);
grid on
xlabel('min'), ylabel('%')
title('percentage coverage over time')
legend('rg algorithm')