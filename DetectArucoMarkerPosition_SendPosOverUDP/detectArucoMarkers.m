function [markerID, markerCornersX, markerCornersY] = detectArucoMarkers(imbw,markerDatabase)
%DETECTMARKERS Detect Aruco Markers in a binary image. 
% (can also be compiled in Simulink, on the AR.Drone 2.0 )
% (this is a different method as the original Aruco marker method)
%
% Syntax:   [markerID, markerCornersX, markerCornersY] = ...
%                                  detectArucoMarkers(imbw,markerDatabase)
%
% Inputs:
%   imbw - Thresholded binary image (use for example: imbw = im2bw(im, 0.5).
%   markerDatabase - List of known Aruco markers, where each column
%                    represents a marker. For example:
%                    if
%                      column = [1 1 0 0 0 0 1 1 0 0 0 1 1 1 0 0]'
%                    then
%                      marker = [1 0 0 1;
%                                1 0 0 1;
%                                0 1 0 0;
%                                0 1 1 0]
%                    thus
%                       column = reshape(marker,[],1)
%                    where a 1 represents a white block 
%
% Outputs:
% 	markerID - Column vector with found markers, each marker ID 
%              corresponds to a column in the markerDatabase. 
%              The size is equal to the number of columns of
%              markerDatabase.
%   markerCornersX - Each column contains the x position of the 4 
%                    corners (Clock wise starting at left top in marker frame).  
%                    The column number corresponds to the column number of markerID.
%                    (the X position is given in width direction of the
%                    image starting from the left)
%   markerCornersY - Each column contains the y position of the 4 
%                    corners of the marker. (Clock wise)
%                    (the Y position is given in the height direction of the
%                    image starting from the top)
%
% Notes:
%       ---- WE STRONGLY RECOMEND TO USE MATLAB CODER ----
% Compiling the code reduces the computation time significantly, 
% run the following code to compile this script: 
%
%           codegen detectArucoMarkers -args {imbw,markerDatabase}
%
% where the arguments 'imbw' and 'markerDatabase' are example matrices 
% indicating the size and format that you want this function to work with.
% (such that the compiler knows what it can expect)
% After compilation use 
%              [] = detectArucoMarkers_mex() 
% instead of 
%              [] = detectArucoMarkers()
% If you not yet have a compiler or you just don't know, then use the code: 
%                           mex -setup 
% and install one.
%
% Another tip to speed up the process is to not use the command 
%   clear all; 
% in the loop.
%
% A solution to some compilation errors might be:
%   clear mex; 
%   clear function; 
% and remove the file 'detectArucoMarkers_mex.mex64' and the folder 'codegen'



%#codegen
% get image size
[h, w] = size(imbw);

% preallocate some variables
contoursize = 0;
contourclosed = false;
contourmaxsize = 2*h+2*w;
contourx = zeros(contourmaxsize,1);
contoury = zeros(contourmaxsize,1);
contourxsum = 0;
contourysum = 0;

maxRatioContourPoints = 8;
maxRatioCornerPoints = 8;

eucdist = zeros(contourmaxsize,1);

markerElements = round(sqrt(size(markerDatabase,1)));
markersDbSize = size(markerDatabase,2);

markerFoundCount = 0;
markerID = zeros(1,markersDbSize);
markerCornersX = zeros(4,markersDbSize);
markerCornersY = zeros(4,markersDbSize);

stop = false;
minRibSize = 5;

startpoint = 1;
stoppoint = 10;
MarkerIdCurrent = 0;
MarkerRot = 0;

% Extend all white contours with one pixel
imdil = imdilate(imbw,[0 1 0; 1 1 1; 0 1 0]);
% The difference gives us the contours
imcontours = imdil & ~imbw;

% Search for contours 
% 1) check if there is a TRUE pixel, starting from the top left of the image 
% 2) if a pixel is found then follow the contour clock wise and set the
%    current pixel to false.
% 3) if the end of the contour is close to the start then we have a closed contour.
% 4) when contour is closed, then calculate the euclidian distance from the 
%    the contour center to each point on the contour.
% 5) the euclidian distance will show four peaks, each corresponding to a
%    corner.
% 6) the peaks are used to separate the contour points in four groups, each
%    representing a line of the square.
% 7) estimate lines from the four groups
% 8) calculate intersections of the lines, this are the marker corners
% 9) calculate the sample positions in the marker interior
% 10) sample and compare the result with the database
% 11) if marker found then add to the matrix
% 12) if not all pixels in an image are false, then jump back to step 1

% Loop to walk to all pixels
for hh = 2:h-1,
    for ww = 2:w-1,
        if imcontours(hh,ww), % check if pixel true            
            stop = false;
            contoursize = 0;
            contourxsum = 0;
            contourysum = 0;
            contourx = zeros(contourmaxsize,1);
            contoury = zeros(contourmaxsize,1);
            hc = hh;
            wc = ww;
            while ~stop, % follow contour
                contoursize = contoursize + 1;
                if contoursize >= contourmaxsize,
                    stop = true;
                    break;
                end
                contoury(contoursize) = hc;
                contourysum = contourysum + hc;
                contourx(contoursize) = wc;
                contourxsum = contourxsum + wc;
                contourclosed = false;
                % the order below is important, it ensures clock wise
                % contour following
                if imcontours(hc,wc+1),
                    imcontours(hc,wc) = false;
                    wc = wc + 1;
                elseif imcontours(hc+1,wc),
                    imcontours(hc,wc) = false;
                    hc = hc + 1;   
                elseif imcontours(hc-1,wc),
                    imcontours(hc,wc) = false;
                    hc = hc -1;
                elseif imcontours(hc,wc-1),
                    imcontours(hc,wc) = false;
                    wc = wc - 1;
                elseif imcontours(hc+1,wc+1),
                    imcontours(hc,wc) = false;
                    hc = hc + 1;
                    wc = wc + 1;
                elseif imcontours(hc-1,wc+1),
                    imcontours(hc,wc) = false;
                    hc = hc - 1;
                    wc = wc + 1;               
                elseif imcontours(hc-1,wc-1),
                    imcontours(hc,wc) = false;
                    hc = hc - 1;
                    wc = wc - 1;                
                elseif imcontours(hc+1,wc-1),
                    imcontours(hc,wc) = false;
                    hc = hc + 1;
                    wc = wc - 1;
                else % end of contour
                    imcontours(hc,wc) = false;
                    stop = true;
                    if (wc-1 == ww || wc+1 == ww || ww == ww) && ...
                            (hc-1 == hh || hc+1 == hh || hh == hh),
                        contourclosed = true;
                    end
                end
                % stop if contour walks out of the image
                if hc==1 || hc==h || wc==1 || wc==w,
                    stop = true;
                end
                
            end
            %figure(3)
            %hold off
            %imshow(imcontours)
            %contourclosed
            
            

            if contourclosed,           
                % check if countour is large enough
                if contoursize > 4*minRibSize,
                    % estimate center of contour
                    mnx = contourxsum/contoursize;
                    mny = contourysum/contoursize;
                    
                    % calculate euclidian distance from center to points on contour
                    eucdist = sqrt((contourx-mnx).^2+(contoury-mny).^2);
              
                    
                    % moving average in 2 directions
                    %ratio = 0.5;
                    ratio = 0.52 - 0.0003*contoursize;
                    ratioC = 1-ratio;
                    eucdist(1) =  ratio*eucdist(1) + ratioC*eucdist(contoursize);
                    for ng = 2:contoursize,
                        eucdist(ng) =  ratio*eucdist(ng) + ratioC*eucdist(ng-1);
                    end
                    eucdist(contoursize) =  ratio*eucdist(contoursize) + ratioC*eucdist(1);
                    for ng = 2:contoursize,
                        eucdist(contoursize+1-ng) =  ratio*eucdist(contoursize+1-ng) + ratioC*eucdist(contoursize+1-ng+1);
                    end
                    % select the first n peaks
                    peaksmaxn = 20;
                    peaks = zeros(peaksmaxn,2);
                    peakscount = 0;
                    if eucdist(1)>eucdist(contoursize) && eucdist(1)>eucdist(2)  && peakscount < peaksmaxn,
                        peakscount = peakscount + 1;
                        peaks(peakscount,1) = 1;
                        peaks(peakscount,2) = eucdist(1);
                    end
                    for ng = 2:contoursize-1,
                        if eucdist(ng)>eucdist(ng-1) && eucdist(ng)>eucdist(ng+1) && peakscount < peaksmaxn ,
                            peakscount = peakscount + 1;
                            peaks(peakscount,1) = ng;
                            peaks(peakscount,2) = eucdist(ng);
                        end
                    end
                    if eucdist(contoursize)>eucdist(1) && eucdist(contoursize)>eucdist(contoursize-1)  && peakscount < peaksmaxn,
                        peakscount = peakscount + 1;
                        peaks(peakscount,1) = contoursize;
                        peaks(peakscount,2) = eucdist(contoursize);
                    end
                    % if more than 4 then select the 4 highest peak
                    peaksselect = zeros(4,1);
                    if peakscount > 4,
                        for mg = 1:4, % set peak value of the 4 higest peaks to -1
                            peakcurrent = [1; 0];
                            for mk = 1:peakscount,
                                if peaks(mk,2) > peakcurrent(2),
                                    peakcurrent(2) = peaks(mk,2);
                                    peakcurrent(1) = mk;
                                end
                            end
                            peaks(peakcurrent(1),2) = -1;
                        end
                        peaksselectcount = 0;
                        for mk = 1:peakscount, % select the peaks with a -1
                            if  peaks(mk,2) == -1,
                                peaksselectcount = peaksselectcount + 1;
                                peaksselect(peaksselectcount) = peaks(mk,1);
                            end
                        end
                    end
                    % if already 4 peaks the select these 4 peaks
                    if peakscount == 4,
                        for mg = 1:4,
                            peaksselect(mg) = peaks(mg,1);
                        end
                    end
                    
                    %figure(4)
                    %hold off
                    %plot(eucdist(1:contoursize))
                    %peaks
                    %peaksselect
                    %contoursize
                    %hold on
%                     if peakscount >= 4
%                         plot(peaksselect,eucdist(peaksselect),'or')
%                     end
% pause
                    
                    
                    % estimate lines
                    if peakscount >= 4,
                        
%                         figure(4)
%                         hold off
%                         plot(eucdist(1:contoursize))
%                         hold on
%                         plot(peaksselect,eucdist(peaksselect),'or')
                        
                        
                        % slope = (n*sum(X.*Y) - sum(X)*sum(Y)) / (n*sum(X.^2)-sum(X)^2)
                        % intercept = mean(Y) - slope*mean(X)
                        
                        X = contourx;
                        Y = contoury;
                        XY = X.*Y;
                        XX = X.*X;
                        
                        
                        paddingfrac = 0.1;
                        lineerror = false;
                        cyclethroughzero = false;
                        distbetweenpeaksPrevious = 0;
                        linepar = zeros(4,3); % each row [lineisvertical, slope, intersect]
                        % for each line
                        for kn = 1:4;                            
                            cyclethroughzero = false;
                            if kn ==4,
                                cyclethroughzero = true;
                                distbetweenpeaks = contoursize-peaksselect(4) + peaksselect(1);
                                padding = round(paddingfrac*distbetweenpeaks);
                                if distbetweenpeaks >= minRibSize
                                    startpoint = peaksselect(4)+padding;
                                    if startpoint > contoursize,
                                        startpoint = startpoint- contoursize;
                                        cyclethroughzero = false;
                                    end
                                    stoppoint = peaksselect(1)-padding;
                                    if stoppoint < 1,
                                        stoppoint = contoursize - stoppoint;
                                        cyclethroughzero = false;
                                    end
                                else
                                    lineerror = true;
                                    break;
                                    %ribsizeTosmall =1
                                end
                            else % if not the forth line
                                distbetweenpeaks = peaksselect(kn+1)-peaksselect(kn);
                                if distbetweenpeaks >= minRibSize,
                                    padding = round(paddingfrac*distbetweenpeaks);
                                    startpoint = peaksselect(kn)+padding;
                                    stoppoint = peaksselect(kn+1)-padding;
                                else
                                    lineerror = true;
                                    break;
                                    %ribsizeTosmall =2
                                end
                            end
                            
                            %throwOut = false
                            % lines must approximatly have the same length
                            if kn>1,
                                lineRatio = distbetweenpeaksPrevious/distbetweenpeaks;
                                if lineRatio > maxRatioContourPoints || lineRatio < 1/maxRatioContourPoints,
                                    lineerror = true;
                                    %throwOut = true
                                    break;
                                end
                            end
                            distbetweenpeaksPrevious = distbetweenpeaks;
                           
                            % estimate lines
                            sumX = 0;
                            sumY = 0;
                            sumXY = 0;
                            sumXX = 0;
                            n = 0;
                            for kv = 1:contoursize,
                                %                                 if kv == 10, pause; end
                                if (cyclethroughzero && (kv<=stoppoint || kv>=startpoint)) ||...
                                        (~cyclethroughzero && kv>=startpoint && kv<=stoppoint),
                                    sumX = sumX + X(kv);
                                    sumY = sumY + Y(kv);
                                    sumXY = sumXY + XY(kv);
                                    sumXX = sumXX + XX(kv);
                                    n = n + 1;
                                    
                                    %                                     plot(X(kv),Y(kv), 'om')
                                    % %                                     pause
                                end
                            end
                            if n>0,
                                % check if line is vertical
                                if abs(n*sumXX-sumX^2)<0.01,
                                    linepar(kn,1) = 1;
                                    linepar(kn,3) = sumX/n;
                                else % if not vertical
                                    linepar(kn,2) = (n*sumXY - sumX*sumY) / (n*sumXX-sumX^2);
                                    linepar(kn,3) = (sumY - linepar(kn,2)*sumX)/n;
                                end
                            else
                                lineerror = true;
                                %break;
                            end
                            
                            
                        end% end for each line
                        %                         figure(3)
                        % %                             hold off
                        %                             imshow(imcontours);
                        %                             hold on
                        %                             plot(contourx(1:contoursize),contoury(1:contoursize));
                        %                             plot(mnx,mny,'or');
                        
                        if ~lineerror,
                        % check if all lines that must cross really cross
                        if linepar(1,1)*linepar(2,1)==1 || linepar(2,1)*linepar(3,1)==1 || ...
                                linepar(3,1)*linepar(4,1)==1 || linepar(4,1)*linepar(1,1)==1,
                            lineerror = true;
%                             linesDoNotCross=1
                        end
                        %lineerror
                        end
                        
                        if ~lineerror, % if lines are ok, calculate line intersections
                            corners = zeros(4,2);
                            for cr = 1:4;
                                if cr == 4,
                                    cr2 = 1;
                                else
                                    cr2 = cr + 1;
                                end
                                % corner 1
                                if linepar(cr,1)==1,
                                    corners(cr,1) = linepar(cr,3);
                                    corners(cr,2) = linepar(cr2,2)*corners(cr,1) + linepar(cr2,3);
                                elseif linepar(cr2,1)==1,
                                    corners(cr,1) = linepar(cr2,3);
                                    corners(cr,2) = linepar(cr,2)*corners(cr,1) + linepar(cr,3);
                                else
                                    corners(cr,1) = (linepar(cr2,3)-linepar(cr,3))/(linepar(cr,2)-linepar(cr2,2));
                                    corners(cr,2) = linepar(cr,2)*corners(cr,1) + linepar(cr,3);
                                end
                            end
                            
                            
                            %% check if distance between corners is approx equal
                            cornersOutOfProportion = false;
                            distbetweenCornersPrevious = sqrt((corners(4,1)-corners(1,1))^2  + (corners(4,2)-corners(1,2))^2);
                            for cp = 1:3,
                                distbetweenCorners = sqrt((corners(cp,1)-corners(cp+1,1))^2  + (corners(cp,2)-corners(cp+1,2))^2);
                                if distbetweenCorners > 0,
                                    ratioCorners = distbetweenCornersPrevious/distbetweenCorners;
                                    if ratioCorners > maxRatioCornerPoints || ratioCorners < 1/maxRatioCornerPoints,
                                        cornersOutOfProportion = true;
                                        break;
                                    end
                                else
                                    cornersOutOfProportion = true;
                                    break;
                                end
                                distbetweenCornersPrevious = distbetweenCorners;
                            end
                                
                            if ~cornersOutOfProportion,  
                            %% Check if it is a Marker
                            markerFound = false;
                            Aruco = zeros(markerElements);
                            ArucoR = zeros(markerElements);
                            for k = 1:markerElements;
                                % select sampling position along the line between corner  1-2 and 3-4
                                % this wil create two new ponts indicated with 'a' and 'b'
                                r = k*1/(markerElements+2)+1/((2*(markerElements+2)));
                                for l = 1:markerElements;
                                    m = l*1/(markerElements+2)+1/((2*(markerElements+2))); % percentage along line a-b
                                    % ax = crn1x + r*(crn2x - crn1x);
                                    % ay = crn1y - r*(crn1y - crn2y);
                                    % bx = crn4x + r*(crn3x - crn4x);
                                    % by = crn4y - r*(crn4y - crn3y);
                                    
                                    ax = corners(1,1) + r*(corners(2,1) - corners(1,1));
                                    ay = corners(1,2) - r*(corners(1,2) - corners(2,2));
                                    bx = corners(4,1) + r*(corners(3,1) - corners(4,1));
                                    by = corners(4,2) - r*(corners(4,2) - corners(3,2));
                                    
                                    % select on the line a-b point c
                                    cx = round(ax + m*(bx - ax));
                                    cy = round(ay + m*(by - ay));
                                    % check if point is within the image
                                    if (cx>1) && (cx<w) && (cy>1) && (cy<h),
                                        % sample pixel and put it in the marker matrix
                                        Aruco(l,k) = imbw(cy,cx);
                                    end
                                end
                            end
                            %% put marker rotations in vector format
                            M = zeros(markerElements^2,4);
                            for vr = 1:4,                                
                                % place marker in column
                                for va = 1:markerElements,
                                    for vb = 1:markerElements
                                        M((va-1)*markerElements+vb,vr) = Aruco(vb,va);   
                                    end
                                end
                                if vr ~=4,
                                    % rotate 90 ccw
                                    for va = 1:markerElements,
                                        for vb = 1:markerElements
                                            ArucoR(markerElements+1-va,vb) = Aruco(vb,va);
                                        end
                                    end
                                    Aruco = ArucoR;
                                end                                
                            end
                            %% Compare markers with database
                            
                            for mid = 1:markersDbSize,
                                for mro = 1:4,
                                    diffcount = 0;
                                    for md = 1:markerElements^2,
                                        if M(md,mro) ~= markerDatabase(md,mid),
                                            diffcount = diffcount + 1;
                                        end
                                    end
                                    if diffcount == 0,
                                        markerFoundCount = markerFoundCount + 1;
                                        markerFound = true;
%                                         if markerFound, 
%                                             markerDatabase
%                                             M
%                                             pause; end
                                        MarkerIdCurrent = mid;
                                        MarkerRot = mro;
                                        break
                                    end
                                end
                                if markerFound,
                                    break
                                end
                            end
                            
                            if markerFound,
                               markerID(1, markerFoundCount) = MarkerIdCurrent; 
                               
                               if markerFound
%                                     cornersSorted =zeros(4,2);
                                    % set the corners in the right order
                                    for j = 1:4,
                                        p = j + (MarkerRot-1);
                                        if p > 4, p = p - 4; end
%                                         cornersSorted(j,1) = corners(p,1);
                                        markerCornersX(j,markerFoundCount) = corners(p,1);
                                        markerCornersY(j,markerFoundCount) = corners(p,2);
                                    end                    
                               end
                               
                            end % end MarkerFound

                            end %~cornersOutOfProportion
                        end
                        
                    end % end find lines
                    
                    
                end % end contour lengt > 4*ribsize
            end % end if contourclosed
        end
    end
end

%  out = minp;
