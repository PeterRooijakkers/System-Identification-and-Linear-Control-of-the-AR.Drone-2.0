function markerMatrix = markerShow(index, markerDatabase, sizeInPixelsPerBit)
elements = round(sqrt(size(markerDatabase,1)));
m = reshape(markerDatabase(:,index),elements,elements);
markerMatrix = [ones(sizeInPixelsPerBit,(elements+4)*sizeInPixelsPerBit);...
    ones(sizeInPixelsPerBit),zeros(sizeInPixelsPerBit,(elements+2)*sizeInPixelsPerBit),ones(sizeInPixelsPerBit)];
e = [];
for i = 1:elements,
    r = [ones(sizeInPixelsPerBit),zeros(sizeInPixelsPerBit)];
    for k = 1:elements,
        if m(i,k)==1,
            e = ones(sizeInPixelsPerBit);
        else
            e = zeros(sizeInPixelsPerBit);
        end
        r = [r,e];
    end
    r = [r, zeros(sizeInPixelsPerBit), ones(sizeInPixelsPerBit)];
    markerMatrix = [markerMatrix; r];
end
markerMatrix = [markerMatrix;... 
        ones(sizeInPixelsPerBit), zeros(sizeInPixelsPerBit,(elements+2)*sizeInPixelsPerBit), ones(sizeInPixelsPerBit);...
        ones(sizeInPixelsPerBit,(elements+4)*sizeInPixelsPerBit)];
    
    markerMatrix = insertText(markerMatrix,[1,1],index,'FontSize',min([70,round(sizeInPixelsPerBit/2)]),'BoxOpacity',0,'TextColor',[0.8 0.8 0.8]);
    markerMatrix = rgb2gray(markerMatrix);
end
