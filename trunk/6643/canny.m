% Matlab program to implement the Canny Edge method
clear all;
close all;

% Specify file attributes for raw data file
filename = 'Lena.raw';
imageW = 256;
imageH = 256;
% Open file pointer for read
id = fopen(filename, 'r');
% Read in the data.
img = fread(id, [imageW,imageH])';
img = double(img);
% Close file pointer
fclose(id);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gaussian Smoothing Starts %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize space for filtered image
filteredImage = zeros(imageW,imageH);
% Define the 7X7 Gaussian filter
filter = [1,3,7,9,7,3,1;
    3,12,26,33,26,12,3;
    7,26,55,70,55,26,7;
    9,33,70,90,70,33,9;
    7,26,55,70,55,26,7;
    3,12,26,33,26,12,3;
    1,3,7,9,7,3,1];
% Dimensions of filter for further operations
[filterH,filterW] = size(filter);
% Filter Center for further operations
filterCenterH = (filterH - 1) / 2;
filterCenterW = (filterW - 1) / 2;

% Convolution of the image and the filter mask
for i = (filterCenterH + 1):(imageH - filterCenterH)
    for j = (filterCenterW + 1):(imageW - filterCenterW)
        filteredImage(i,j)=sum(sum(img(i-filterCenterH:i+filterCenterH,j-filterCenterW:j+filterCenterW).*filter));
    end
end
% Normalize Image
fmax = max(max(filteredImage)); % Max value in the matrix
fmin = min(min(filteredImage)); % Min value in the matrix
for i = 1:imageH
    for j = 1:imageW
        % Normalizing function
        filteredImage(i,j) = round((filteredImage(i,j)-fmin)*255/(fmax-fmin));
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gaussian Smoothing Ends %
%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Operation with Roberts Operator Starts %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define Roberts Operator
Mx = [0,1;-1,0];
My = [1,0;0,-1];
% Initiate Mag and Dir matrices
Mag = zeros(imageH,imageW);
Dir = zeros(imageH,imageW);
% Initialize local variables
Gx = 0;
Gy = 0;

% Run both filters and derive Mag,Dir values
for i = 1:(imageH-1)
    for j = 1:(imageW-1)
        Gx = sum(sum(filteredImage(i:i+1,j:j+1).*Mx));
        Gy = sum(sum(filteredImage(i:i+1,j:j+1).*My));
        Mag(i,j) = sqrt( Gx^2 + Gy^2 );
        % Whenever the gradient in the x direction is equal to zero, the
        % edge direction has to be equal to 90 degrees or 0 degrees, depending on what the value of the gradient in
        % the y-direction is equal to. If GY has a value of zero,
        % the edge direction will equal 0 degrees. Otherwise the edge direction will equal 90 degrees.
        if Gx == 0
            if Gy == 0
                Dir(i,j) = 0;
            else
                Dir(i,j) = pi/2;
            end
        else
            Dir(i,j) = atan(Gy/Gx);
        end
    end
end
% Convert to degrees
Dir = Dir .* (180/pi);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Operation with Roberts Operator Ends %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Non Maximum Supression Starts %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initiate NonMaxImage to Mag
NonMaxImage = Mag;

% Loop through the image pixels
for i = 2:(imageH-1) % avoiding edge case as that will cause Matrix OOB
    for j = 2:(imageW-1)
        % Supressing any pixel response which is not higher than two
        % neighbouring pixels on either side of it along direction of gradient

        % for edge along (-1,0) and (1,0)
        if ( ( Dir(i,j) < 22.5 ) && ( Dir(i,j) > -22.5 ) )
            if ( ( Mag(i,j) < Mag(i-1,j) ) || Mag(i,j) < Mag(i+1,j) )
                NonMaxImage(i,j) = 0;
            end

            % for edge along (-1,-1) and (1,1)
        elseif ( ( Dir(i,j) > 22.5 ) && ( Dir(i,j) < 67.5 ) )
            if ( ( Mag(i,j) < Mag(i-1,j-1) ) || Mag(i,j) < Mag(i+1,j+1) )
                NonMaxImage(i,j) = 0;
            end

            % for edge along (-1,1) and (1,-1)
        elseif ( ( Dir(i,j) < -22.5 ) && ( Dir(i,j) > -67.5 ) )
            if ( ( Mag(i,j) < Mag(i-1,j+1) ) || Mag(i,j) < Mag(i+1,j-1) )
                NonMaxImage(i,j) = 0;
            end

            % for edge along (0,1) and (0,-1)
        else
            if ( ( Mag(i,j) < Mag(i,j+1) ) || Mag(i,j) < Mag(i,j-1) )
                NonMaxImage(i,j) = 0;
            end
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Non Maximum Supression Ends %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Contour Tracking Starts %
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initiate edge image matrix to all zeros
edge = zeros(imageH,imageW);
% Define low and high threshold values
threshH = 25;
threshL = 5;

% Loop through the image pixels
for i = 2:(imageH-1) % avoiding edge case as that will cause Matrix OOB
    for j = 2:(imageW-1)
        % Look for pixel whose value is greater or equal to higher threshold
        if ( NonMaxImage(i,j) >= threshH )
            edge(i,j) = 1;
            % Loop through the 8-neighbour pixels
            for k = [-1,1]
                for l = [-1,1]
                    % Look for pixel whose value is greater than lower threshold
                    if ( NonMaxImage(i+k,j+k) > threshL )
                        edge(i+k,j+k) = 1;
                    end
                end
            end
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%
% Contour Tracking Ends %
%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%
% Display the results %
%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,3,1);imshow(uint8(img));title('Original Image');
subplot(2,3,2);imshow(uint8(filteredImage));title('Gaussian Filter');
subplot(2,3,3);imshow(uint8(Mag));title('Magnitude Image');
subplot(2,3,4);imshow(uint8(Dir));title('Directional Image');
subplot(2,3,5);imshow(uint8(NonMaxImage));title('Non Maxima Supressed Image');
subplot(2,3,6);imshow(edge);title('Edge Image');
