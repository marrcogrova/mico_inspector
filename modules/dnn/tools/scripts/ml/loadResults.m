function [results] = loadResults(filename)

raw = load(filename);

M = raw(1); % Or raw(4);
k = raw(2);
results = [];
subRaw = raw(3:end);
for d = 1:(M-1)
    for t = 1:k
        results(d,t) = subRaw(t + d*k);
    end 
end
end