%% Noise Function - Rick 17-07-2021

function d = noise()

x = 0; x1 = 0; x2 = 0; x3 = 0; x4 = 0; nu = 0; d = [];
constant = 5.0; max = 0; 
xmin=0; xmax=1; n=1;
rand_x = xmin+rand(1,n)*(xmax-xmin);

for i = 1:1:400
    rand_x = rand_num();
    x =  0.5 - rand_x;
    x1 = x1+(x-x1)/constant;
    x2 = x2+(x1-x2)/constant;
    x3 = x3+(x2-x3)/constant;
    x4 = x4+(x3-x4)/constant;
end

for i = 1:1:800
    rand_x = rand_num();
    x = 0.5 - rand_x;
    x1 = x1+(x-x1)/constant;
    x2 = x2+(x1-x2)/constant;
    x3 = x3+(x2-x3)/constant;
    x4 = x4+(x3-x4)/constant;
    d(i) = (10000*x4);
end

for i = 1:1:800
    if (d(i) > max)
        max = abs(d(i));
    end
end

for i = 1:1:800
    d(i)  = d(i)/(max) * 100.0;
end
    
end