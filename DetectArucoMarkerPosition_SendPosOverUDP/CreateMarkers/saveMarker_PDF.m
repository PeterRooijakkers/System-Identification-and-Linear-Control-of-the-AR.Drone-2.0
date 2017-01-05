% SAVE MARKERS IN PDF FORMAT

% select marker
for index = 1:5,
    
    % Scaling factor such that the figure fits on the screen,
    % take this into account during printing.
    scale = 0.5;
    
    % load data base
    load('markerDatabase_4x4x1000');
    
    % create marker
    sizeInPixelsPerBit = 300;
    markerImage = markerShow(index, markerDatabase_4x4x1000, sizeInPixelsPerBit);
    
    markerFigure = figure(10);
    % use cm
    set(markerFigure, 'Units','centimeters')
    
    % A4 size
    height1 = 29.7*scale;
    width = 21.0*scale;
    
    % set figure properties
    set(markerFigure,'color','W',...
        'Position',[0 0 width height1],...
        'PaperSize',[width height1],...       %'PaperPositionMode','auto',...
        'InvertHardcopy', 'off',...
        'Renderer','painters');
    
    % set marker size
    imsizeBlackSquare = 10.0*scale; % cm
    % do some scaling to take into account the white border for 4x4 markers
    imheightmarker = imsizeBlackSquare/6*8;
    imwidthmarker = imsizeBlackSquare/6*8;
    % center marker on paper
    imtop = (height1-imheightmarker)/2;
    imleft = (width-imwidthmarker)/2;
    
    % set properties of subplot where we put the marker in
    subplot(1,1,1,'Parent', markerFigure,'Units','centimeters','position',[imleft imtop imwidthmarker imheightmarker]);
    
    % plot marker
    imshow(markerImage)
    
    % save as pdf
    hgexport(markerFigure, strcat('marker', num2str(index), '.pdf'), hgexport('factorystyle'), 'Format', 'pdf');
    
end
