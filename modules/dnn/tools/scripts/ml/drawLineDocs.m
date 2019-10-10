function [] = drawLineDocs(theta,gt)

vertexs = [ 0 0 0;
            1 0 0];
       
%figure();

[M K] = size(theta);

points = [];
for m = 1:M
   docProb = theta(m,:);
   point = docProb*vertexs;
   points = [points ; point];
end

scatter3(points(:,1),points(:,2),points(:,3));
hold on;
scatter3(vertexs(:,1),vertexs(:,2),vertexs(:,3), 'r*');

end