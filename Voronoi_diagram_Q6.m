
clear all
clc
x=[224 183 362 262 614 672 869 622];
y=[62 210 248 418 62 294 294 452];
plot(x,y,'o');
axis([-100 1000 -100 500]);
hold on;
voronoi(x,y)