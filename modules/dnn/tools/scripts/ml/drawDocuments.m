function [] = drawDocuments(theta, gt)

[n m] = size(theta);

switch(m)
    case 2
        drawLineDocs(theta, gt);
    case 3
        drawTriangleDocs(theta, gt);
    case 4
        drawTetraDocs(theta, gt,'o');
end

end