%% random Number generator between 0 and 1

function rand_x = rand_num()
xmin=0; xmax=1; n=1;
rand_x = xmin+rand(1,n)*(xmax-xmin);
end