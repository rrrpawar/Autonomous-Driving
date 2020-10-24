clear all
u = arduino()
% Set up
%u = arduino('COM6', 'Uno', 'Libraries', 'Servo'); % Check COM#
v = servo(u,'D13', 'MinPulseDuration', 1e-3, 'MaxPulseDuration', 2e-3);
s = servo(u,'D12', 'MinPulseDuration', 1e-3, 'MaxPulseDuration', 2e-3);
z = 2;
i = 1;
theta = 0;
p(1) = 0.5
X1 = 0;
Y1 = 0;
X2 = 0;
Y2 = 0;
X3 = 0;
Y3 = 0;
X4 = 0;
Y4 = 0;
P0 = 0.1;
Q = 0.03;
R0 = 0.01;
K = 0.5;
u0 = 323.5;
v0 = 172.5;
Fu = 188.58;
Fv = 99.94;
Zc = 0.33; %check
X=[1 0 0 0;0 0.88 -0.47 0;0 0.47 0.88 0.33;0 0 0 1];   %extrinsic matrix
inv_X = inv(X); %inverse of X
cam = webcam('USB_Camera');
cam.resolution = '640x480';
% mov=VideoReader('tiral_new.mp4');
while (1)
pic = snapshot(cam);
% % pic = imresize(pic,0.4);    
% pic = readFrame(mov);
%pic = imresize(pic,0.2);
figure(1)
imshow(pic);    
hold on
shape = size(pic);
% figure(1); imshow(pic);
gray_pic = rgb2gray(pic);
% figure(2); imshow(gray_pic);
edge_pic = edge(gray_pic,'canny',[0.31,0.36]);
% figure(3); imshow(edge_pic);
%edge detection
%region masking by selecting a polygon
%specify row coordinates of polygon

% if (theta<=-0.18)
%     a=[shape(2), shape(2), shape(2), 0];
%     %specify column coordinates of polygon
%     b=[shape(1)*0.8, shape(1)*0.8, shape(1),shape(1)];
% else
%     a=[shape(2)*0.2, shape(2)*0.8, shape(2), 0];
%     %specify column coordinates of polygon
%     b=[shape(1)*0.6, shape(1)*0.6, shape(1),shape(1)];
% end

%leftlines
a=[shape(2)*0.2, shape(2)*0.3, shape(2)*0.5, 0];
%specify column coordinates of polygon
b=[shape(1)*0.6, shape(1)*0.6, shape(1),shape(1)];
polyshape(a,b);
% a=[shape(2)*0.2, shape(2)*0.8, shape(2), 0];
% %specify column coordinates of polygon
% b=[shape(1)*0.6, shape(1)*0.6, shape(1),shape(1)];
% plot(draw_rx,draw_y,'LineWidth',2,'Color','red');
bw=roipoly(pic,a,b);
BW=(edge_pic(:,:,1)&bw);
% imshow(BW);
%hough line detection
[H,T,R] = hough(BW);
% P=houghpeaks(H,1,'Threshold',0.3*ceil(max(H(:))));
P=houghpeaks(H,1);
lines = houghlines(BW,T,R,P,'FillGap',50,'MinLength',5);
% imagesc(pic);
anglethres=0.2; %separate left/right by orientation threshold
leftlines=[];rightlines=[]; %Two group of lines
for k = 1:length(lines)
x1=lines(k).point1(1);
y1=lines(k).point1(2);
x2=lines(k).point2(1);
y2=lines(k).point2(2);
%     if (x2>=shape(2)/2) && ((y2-y1)/(x2-x1)>anglethres)
%     rightlines=[rightlines;x1,y1;x2,y2];
%     elseif (x2<=shape(2)/2) && ((y2-y1)/(x2-x1)<(-1*anglethres))
%     leftlines=[leftlines;x1,y1;x2,y2];
%     end
end

    P0 = P0+Q;
    K = P0/P0+R0;
    x1 = X1+K.*(x1-X1);
    y1 = Y1+K.*(y1-Y1);
    x2 = X2+K.*(x2-X2);
    y2 = Y2+K.*(y2-Y2);
    P0 = (1-K)*P0;
    X1 = x1;
    Y1 = y1;
    X2 = x2;
    Y2 = y2;
leftlines=[x1,y1;x2,y2];

%rightlines
a=[shape(2)*0.8, shape(2)*0.7, shape(2)*0.5, shape(2)];
%specify column coordinates of polygon
b=[shape(1)*0.6, shape(1)*0.6, shape(1),shape(1)];

% a=[shape(2)*0.2, shape(2)*0.8, shape(2), 0];
% %specify column coordinates of polygon
% b=[shape(1)*0.6, shape(1)*0.6, shape(1),shape(1)];
% plot(draw_rx,draw_y,'LineWidth',2,'Color','red');
bw=roipoly(pic,a,b);
BW=(edge_pic(:,:,1)&bw);
% imshow(BW);
%hough line detection
[H,T,R] = hough(BW);
% P=houghpeaks(H,1,'Threshold',0.3*ceil(max(H(:))));
 P=houghpeaks(H,1);
lines = houghlines(BW,T,R,P,'FillGap',50,'MinLength',5);
% imagesc(pic);
anglethres=0.2; %separate left/right by orientation threshold
% leftlines=[];rightlines=[]; %Two group of lines
for k = 1:length(lines)
x3=lines(k).point1(1);
y3=lines(k).point1(2);
x4=lines(k).point2(1);
y4=lines(k).point2(2);
%     if (x2>=shape(2)/2) && ((y2-y1)/(x2-x1)>anglethres)
%     rightlines=[rightlines;x1,y1;x2,y2];
%     elseif (x2<=shape(2)/2) && ((y2-y1)/(x2-x1)<(-1*anglethres))
%     leftlines=[leftlines;x1,y1;x2,y2];
%     end
end

    P0 = P0+Q;
    K = P0/P0+R0;
    x3 = X3+K*(x3-X3);
    y3 = Y3+K*(y3-Y3);
    x4 = X4+K*(x4-X4);
    y4 = Y4+K*(y4-Y4);
    P0 = (1-K)*P0;
    X3 = x3;
    Y3 = y3;
    X4 = x4;
    Y4 = y4;

rightlines=[x3,y3;x4,y4];

check_left = isempty(leftlines);
check_right = isempty(rightlines);
% if ((check_left==1)&&theta>0.1)
%     writePosition(v, 0.55);
%     writePosition(s,0.3);
%     return
% end
% 
% if ((check_right==1)&&theta<-0.1)
%     writePosition(v, 0.55);
%     writePosition(s,0.7);
%     return
% end

if ((check_left==1))
    writePosition(v, 0.55);
    writePosition(s,0.3);
    return
end

if ((check_right==1))
    writePosition(v, 0.55);
    writePosition(s,0.7);
    return
end
% E = leftlines;
% F = rightlines;

draw_y=[shape(1)*0.6,shape(1)]; %two row coordinates
PL=polyfit(leftlines(:,2),leftlines(:,1),1);
draw_lx=polyval(PL,draw_y); %two col coordinates of left line
PR=polyfit(rightlines(:,2),rightlines(:,1),1);
draw_rx=polyval(PR,draw_y); %two col coordinates of right line

%     P0 = P0+Q;
%     K = P0/P0+R0;
%     draw_lx = X1+K.*(draw_lx-X1);
%     draw_rx = Y1+K.*(draw_rx-Y1);
%     X1 = draw_lx;
%     Y1 = draw_rx;
%     P0 = (1-K)*P0;
% imagesc(pic);
% plot([a(1),b(1)],[a(2),b(2)],[a(3),b(3)],[a(4),b(4)],'LineWidth',2,'Color','green');
% plot([x1,x2],[y1,y2],'LineWidth',2,'Color','red');
% plot([x3,x4],[y3,y4],'LineWidth',2,'Color','red');
 plot(draw_lx,draw_y,'LineWidth',2,'Color','red');
 plot(draw_rx,draw_y,'LineWidth',2,'Color','red');
 plot([(draw_lx(1)+draw_rx(1))/2,(draw_lx(2)+draw_rx(2))/2],[draw_y]);
%plot([(draw_rx(1)+draw_lx(1))/2,draw_y(1)],[(draw_lx(2)+draw_rx(2))/2,draw_y(2)],'LineWidth',2,'Color','red');
%pixel to camera coordnates
    lx1 = Zc*(draw_lx(1)-u0)/Fu;
    lx2 = Zc*(draw_lx(2)-u0)/Fu;
    rx1 = Zc*(draw_rx(1)-u0)/Fu;
    rx2 = Zc*(draw_rx(2)-u0)/Fu;
    y1 = Zc*(draw_y(1)-v0)/Fv;
    y2 = Zc*(draw_y(2)-v0)/Fv;
    
%camera to world frame
Xlf = inv_X(1,:)*[lx1 y1 Zc 1]';
Xln = inv_X(1,:)*[lx2 y2 Zc 1]';
Xrf = inv_X(1,:)*[rx1 y1 Zc 1]';
Xrn = inv_X(1,:)*[rx2 y2 Zc 1]';
Y1 = inv_X(2,:)*[lx1 y1 Zc 1]';
Y2 = inv_X(2,:)*[lx2 y2 Zc 1]';

% bypass conversion
% Xlf = draw_lx(1);
% Xrf = draw_rx(1);
% Xln = draw_lx(2);
% Xrn = draw_rx(2);
% Y1 = draw_y(1);
% Y2 = draw_y(2);

Xcf = (Xlf+Xrf)/2;
Ycf = Y1;
Xcn = (Xln+Xrn)/2;
Ycn = Y2;

Cpos = 0.12;
theta = -atan2((Ycf-Ycn),(Xcf-Xcn))-1.565-0.015+0.007+0.04;
% ams =Xcf-Xcn
% a(i) =Xcf-Xcn;
% disp(theta);
efa = Xcn+(((Xcf-Xcn)/(Ycf-Ycn))*(Ycn-Cpos));
k1 = 0.01;
% k2 = 0.01;
k2 = 0.018;
p(z) = 0.482+k1*theta+k2*efa;
if (p(z)<=0.2)
    p(z) = 0.2;
elseif (p(z)>=0.8)
    p(z) = 0.8;
end
% disp(p(z))
fprintf('%f %f \n',theta,efa);
% disp(theta)
% disp(efa)
% disp(theta)
% Hold ESC button down until light turns red then release
writePosition(v,0.55)
writePosition(s,p(z))
pause(0.03) 
writePosition(v, 0.5);
writePosition(s,p(z));
pause(0.03)
% writePosition(v, 0.5);
% writePosition(s, 0.482);
%  pause(0.03)
z = z+1;
hold off
end