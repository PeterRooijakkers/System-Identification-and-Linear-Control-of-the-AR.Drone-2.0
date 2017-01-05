% Detect Aruco Marker number 1 in a webcam image and send the real-world 
% positon to the AR.Drone 2.0 using UDP
%
% Run this function in a new Matlab window parallel with the Simulink model 
%
%   DONT FORGET to compile the function 'detectArucoMarkers' by running
%
%       codegen detectArucoMarkers -args {imbw,markerDatabase}
%
%   ( for more info see header in 'detectArucoMarkers' )
%
% Tips:
%   Keep camera Exposure time small to avoid motion blur
%   Comment out the plotting of the live stream to speed up the process


% settings
focalLength = 820; % [px]
cameraHeightAboveQuad = 2.4-0.6; % [m] (approximately)

% load database with Aruco markers
load('markerDatabase_4x4x1000');
markerDatabase = markerDatabase_4x4x1000(:,4:4);

% initialize UDP connection
hudps = dsp.UDPSender('RemoteIPAddress','192.168.1.1','RemoteIPPort',24000); % Drone ip-adress and port number

% clear possible old webcam object
clear cam

% show all webcams and select one
webcamlist
cam = webcam(1);
% cam.Resolution = '320x240';
% cam.Resolution = '1280x960';
cam.Resolution = '640x480';
cam

% neglect the first few images because they can be bad
for n = 1:20;
    img = snapshot(cam);
    pause(0.1)
end

% determine threshold level (value that divides the image histogram in two groups)
im = rgb2gray(img);
level = graythresh(im);
[imh, imw] = size(im);

% Plot marker in image (debug purpose)
hold off
imshow(im)

% initial values
psi = 0;
Xpos = 0;
Ypos = 0;

% run infinite loop
runCapture = 1;
while runCapture == 1 ;
    
    % Capture new image from webcam
    img = snapshot(cam);
    
    % Convert to Grayscale
    im = rgb2gray(img);
    
    % Threshold convert to binary image
    imbw =  im2bw(im, level);
    
    % Detect all aruco markers in the image
    % don't forget to compile, execute the following command in the terminal:
    %        codegen detectArucoMarkers -args {imbw,markerDatabase}
    % tip: make sure the example arguments {imbw,markerDatabase} are available in the workspace
    [markerID, markerCornersX, markerCornersY] = detectArucoMarkers_mex(imbw,markerDatabase);
    
    % Estimate position
    markerfound = false;
    if markerID(1) == 1, % check if marker is found
        markerfound = true;
        cYpix = markerCornersX(:,1) - 0.5*imw;
        cXpix = -(markerCornersY(:,1) - 0.5*imh); % (-) to right hand frame
        % point between corner 1 and 2
        p12Xpix = (cXpix(2)-cXpix(1))/2 + cXpix(1);
        p12Ypix = (cYpix(1)-cYpix(2))/2 + cYpix(2);
        % point between corner 3 and 4
        p34Xpix = (cXpix(4)-cXpix(3))/2 + cXpix(3);
        p34Ypix = (cYpix(3)-cYpix(4))/2 + cYpix(4);
        % estimate angle
        psi = atan2((p12Ypix-p34Ypix), (p12Xpix-p34Xpix));% + 0.5*pi;
        
        % x-y position
        Xpix = mean(cXpix);
        Ypix = mean(cYpix);
        
        % estimate position in meters
        Xpos = Xpix/focalLength*cameraHeightAboveQuad;
        Ypos = Ypix/focalLength*cameraHeightAboveQuad;
    end
    
    % send data via UDP
    datasend = [Xpos,Ypos,psi];
    step(hudps,datasend)
    
    % print result in matlab terminal
    if markerfound,
        display([' x: ' num2str(Xpos) '  y: ' num2str(Ypos) '  psi: ' num2str(psi/pi*180)])
    end
    toc
    tic
    
    % Plot marker in image (debug purpose)
    hold off
    imshow(im)
    hold on
    plot(markerCornersX(:,1),markerCornersY(:,1),'-r', 'linewidth' , 4)

end