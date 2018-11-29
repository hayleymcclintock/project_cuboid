

THRESHOLD_STD = 1.7;    % number between 1 and 2 probabaly. Higher = more leniant in deciding what elevations to include in object 

%% Read in data
data = csvread('/home/robert/project_cuboid/src/control/master_controller/data/L-dark/data/csvSave.csv'); % user input the path to the csv file
% first col is row, 2nd col is col, 3rd is value 

first_side = data(1:2000, :); 
second_side = data(2001:4000, :); 
third_side = data(4001:6000, :); 
fourth_sie = data(6001:8000, :); 

avg_image = mean([first_side(:,3), second_side(:,3), third_side(:,3), fourth_sie(:,3)],2);

mat= reshape(avg_image', 50,40);

% the sensory active square area is 8x8 cm = 80x80mm 
% our patch tkaes 50x40 pixel area so its not square. Assume 50 pixel
% covers one whole 80 mm dimension. Thus, 40 pixel = 64 mm. 

%% Smooth and plot the raw data 
b_smooth = imfilter(mat,fspecial('average',[4 4]));   % smoothing filter 

% plot the raw data that has been smoothed slightly 
hold on; 
a= surf(b_smooth); % also plot as surface to get better res 
shading interp 
xlabel('x pixel'); ylabel('y pixel'); zlabel('Perceived height');
ax = gca; ax.FontSize = 20; 
ax.FontWeight = 'bold';
ax.LineWidth = 2; 
box(ax,'on')
title('raw smoothed'); 
hold off; 

%% processing algorithm to compare to raw 
figure(); hold on; 
xlabel('x pixel'); ylabel('y pixel'); zlabel('Perceived height');
ax = gca; ax.FontSize = 20; 
ax.FontWeight = 'bold';
ax.LineWidth = 2; 
box(ax,'on')
title('smoothed with planes'); 

% 0) cut area of interst to include only things above the minimum (cut out
% perefierie) 
b_smooth = b_smooth(3:45,3:35);
a= surf(b_smooth); % also plot as surface to get better res 
shading interp 

% 1) find the floor (usually the most occuring values that are not within
% certan range of the max which would be the object) and plot it 

floor = mean(mode(b_smooth)); 
ref = surf(ones(50,40)*floor);  % put in the "floor" surface 

% 2) find maxpoint ie, where object probably is 
maximum = max(max(b_smooth)); 
surf(ones(50,40)*maximum); 


% 3) separate everything near the floor plane from the max plane 
store_obj = zeros(size(b_smooth,1),size(b_smooth,2)); 
store_floor = zeros(size(b_smooth,1),size(b_smooth,2));

for i = 1:size(b_smooth,1)
    for j = 1:size(b_smooth,2) 
        if b_smooth(i,j) > maximum - THRESHOLD_STD*max(std(b_smooth)) % if its bigger than max minus threshold, then consider it part of obj. 
            store_obj(i,j) = b_smooth(i,j); 
        end
    end
end

% 4)  normalize object point location with respect to floor (so we can assume 0
% is floor); 
store_obj = store_obj - floor; 
store_obj(store_obj<0) = 0; %and map those negative values back to the floor of 0 


% % FOLLOWING SECTION SHOULD BE COMENTED OUT IF NOT FLAT TOP OBJ%%%%%%%%%%%%
% % 5) find all non zero indicies and smooth across those (to level the tops of
% % object) ONLY FOR FLAT OBJECTS 
% linind = find(store_obj > 0 ); % find the location ofthe object amidst the 0 floor plane
% mean_height =  mean(store_obj(linind)); % calculates the average height 
% 
% % 6) now replace all non zero entires with that constant height 
% for i = 1:size(store_obj,1)
%     for j = 1:size(store_obj,2) 
%         if store_obj(i,j) > 0 
%             store_obj(i,j) = mean_height; 
%         end
%     end
% end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% FOLLOWING SECTION SHOULD BE USED IF THERE AREW MORE THAN 1 DIFFERENT
% HEIGHT OBJECTS 
linind = find(store_obj > 0 ); % find the location ofthe object amidst the 0 floor plane
mean_height =  mean(store_obj(linind)); % calculates the average height 

hold off; 
figure(); title('Final thresholded'); hold on; 
xlabel('x pixel'); ylabel('y pixel'); zlabel('Perceived height');
ax = gca; ax.FontSize = 20; 
ax.FontWeight = 'bold';
ax.LineWidth = 2; 
box(ax,'on')

surf(store_obj); 
shading interp 
surf(zeros(50,40)); % put in actual floor as 0 s to bring back to original camera size 

% im regional max to identify the to peaks 
immax = imregionalmax(store_obj, 26);   % calculate local max with specified connectvity

for i= 1:size(store_obj, 1)
    for j = 1:size(store_obj, 2)
        if immax(i,j) == 1
            plot3(j,i,store_obj(i,j), 'r*')
        end
    end
end

% conver to binary black and white image where the object is 
binary_store_obj = store_obj~=0; 

% fill holes in it 
%binary_store_obj = imfill(binary_store_obj, 'holes'); % fill in the holes to make continuous objects

L = bwlabel(binary_store_obj, 8); % returns mat. each pixel is labeled a number. 1 is 1st obj. 2 is 2nd obj. 3 is 3rd obj. and so on. 

uniques = unique(L(L~=0)); % find out how many numbers are in L, don't include 0, whichisnt a blob 


final_surf = zeros(size(store_obj,1), size(store_obj, 2)); 
for i = 1:length(uniques) % for all the unique blobs (objects in the image) 
    found = find(L == uniques(i));   % get lienar indecies of where that object is       
    mean_found = mean(store_obj(found)); % calculate mean of points in that region 
    
    
    final_surf(found) = mean_found;  % set the final surface value equal to the mean of that area. 
end

% now plot this final surface plot with the relative heights all worked out
% 
figure; 
title('Final thresholded2 '); hold on; 
xlabel('x pixel'); ylabel('y pixel'); zlabel('Perceived height');
ax = gca; ax.FontSize = 20; 
ax.FontWeight = 'bold';
ax.LineWidth = 2; 
box(ax,'on')
surf(final_surf );
shading interp 
surf(zeros(50,40)); % and add the floor back. 
hold off; 

%RGB = label2rgb(L,'hsv', 'k', 'shuffle'); % convert each label of the above L into an RGB code 
%B = bwboundaries(binary_store_obj); % traces the bw boundaries in green             
            

