mov=VideoReader('front_crop.mp4');
while hasFrame(mov)
pic=readFrame(mov);
% undistortedPoints = undistortPoints(pic,cameraParams);
% [undistortedI, newOrigin] = undistortImage(distortedI,cameraParams, 'OutputView', 'full');
shape = size(pic);
% figure(1); imshow(pic);
gray_pic = rgb2gray(pic);
%figure(2); imshow(gray_pic);
edge_pic = edge(gray_pic,'canny',[0.31,0.36]);
figure(3); imshow(edge_pic);

%change resolution
%region masking by selecting a polygon
%specify row coordinates of polygon
a=[shape(2)*0.4, shape(2)*0.6, shape(2), 0];
%specify column coordinates of polygon
b=[shape(1)*0.6, shape(1)*0.6, shape(1),shape(1)];  %vary first two values to change polygon size
bw=roipoly(pic,a,b);
BW=(edge_pic(:,:,1)&bw);
imshow(BW);
%hough line detection
[H,T,R] = hough(BW);
P=houghpeaks(H,3);
lines = houghlines(BW,T,R,P,'FillGap',4,'MinLength',5);
imagesc(pic);
hold on;

% for i= 1:length(lines)
% plot([lines(i).point1(1),lines(i).point2(1)],[lines(i).point1(2),lines(i).point2(2)],'LineWidth',2,'Color','red');
% end

anglethres=0.01; %separate left/right by orientation threshold
 leftlines=[];rightlines=[]; %Two group of lines
for k = 1:length(lines)
    x1=lines(k).point1(1);
    y1=lines(k).point1(2);
    x2=lines(k).point2(1);
    y2=lines(k).point2(2);
    if (x2>=shape(2)/2) && ((y2-y1)/(x2-x1)>anglethres)
        rightlines=[rightlines;x1,y1;x2,y2];
    elseif (x2<=shape(2)/2) && ((y2-y1)/(x2-x1)<(-1*anglethres))
        leftlines=[leftlines;x1,y1;x2,y2];
    end
end

draw_y=[shape(1)*0.6,shape(1)]; %two row coordinates
PL=polyfit(leftlines(:,2),leftlines(:,1),1);
draw_lx=polyval(PL,draw_y); %two col coordinates of left line
PR=polyfit(rightlines(:,2),rightlines(:,1),1);
draw_rx=polyval(PR,draw_y); %two col coordinates of right line
imagesc(pic);
hold on;
plot(draw_lx,draw_y,'LineWidth',2,'Color','red');
hold on
plot(draw_rx,draw_y,'LineWidth',2,'Color','red');
Xcf = (draw_rx(1)+draw_lx(1))/2;
Ycf = draw_y(1);
Xcn = (draw_rx(2)+draw_lx(2))/2;
Ycn = draw_y(2);
Cpos = 0.1;
theta = -atan((Ycf-Ycn)/(Xcf-Xcn))-90;
efa = Xcn+(((Xcf-Xcn)/(Ycf-Ycn))*(Ycn));
k1 = 0.1;
k2 = 0.1;
s = k1*theta+k2*efa;

%a = arduino()

end