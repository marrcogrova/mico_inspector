function [] = drawTetraDocs(theta,gt, displayType);

colors = {strcat('b'), strcat('m'), strcat('g'), strcat('k')};

vertexs = [ 1   0           -1/sqrt(2);
           -1   0           -1/sqrt(2);
           0, +1, 1/sqrt(2);
           0, -1, 1/sqrt(2)];
       
%figure();

[M K] = size(theta);
length(gt);

points = [];
for m = 1:M
   docProb = theta(m,:);
   point = docProb*vertexs;
   points = [points ; point];
end

plot3(vertexs([1,2],1),vertexs([1,2],2),vertexs([1,2],3));
hold on;
plot3(vertexs([1,3],1),vertexs([1,3],2),vertexs([1,3],3));
plot3(vertexs([1,4],1),vertexs([1,4],2),vertexs([1,4],3));
plot3(vertexs([2,3],1),vertexs([2,3],2),vertexs([2,3],3));
plot3(vertexs([2,4],1),vertexs([2,4],2),vertexs([2,4],3));
plot3(vertexs([3,4],1),vertexs([3,4],2),vertexs([3,4],3));

for i=1:(length(gt)-1)
    scatter3(points(i,1),points(i,2),points(i,3), 10, colors{gt(i)+1}, "filled");
end




end