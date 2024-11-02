function u = negPos(x,y)

%initialise index store for positive swap index
index = 0;
%loop through y to find first positive index
for i = 2:1:length(y)+1
    if y(i-1)>0
        index = i+1;
        break
    end    
end

y1 = y(index-1);
y2 = y(index);
y0 = 0;

x1 = x(index-1);
x2 = x(index);

u = x1+((y0-y1)*((x2-x1)/(y2-y1)));

end
