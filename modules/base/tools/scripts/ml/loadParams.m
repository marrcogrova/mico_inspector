function [phi, theta] = loadParams(filename)

raw = load(filename);

k = raw(1); % Or raw(4);
V = raw(2);
M = raw(3);
phi = [];
subRaw = raw(5:(5+k*V - 1));
for t = 1:k
    for v =1:V
        phi(t,v) = subRaw(v + (t-1)*V);
    end 
end

%phi = reshape(raw(5:(4+k*V)), k, V);
theta = [];
subRaw = raw((4 + k*V+1):(4 +  k*V + M*k));
for d = 1:M
    for t = 1:k
        theta(d,t) = subRaw(t + (d-1)*k);
    end 
end
end