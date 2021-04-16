function [c] = place(b,a)
dx=b(1)-a(1);
dy=b(2)-a(2);
r=2;
d2=dx*dx+dy*dy;
if (d2>0)
     a2 = (r + r) ^ 2;
      x = 0.5;
      y = sqrt(max(0, a2 / d2 - x * x));
      c=[a(1) + x * dx - y * dy,a(2) + x * dy + y * dx];
else
    c=[a(1) + r,a(2)];
end
end

