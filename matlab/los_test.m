x=1;
y=100;
th = 0;

xstart = 75;
ystart = 0;
xend = -75;
yend = 0;

[d,lv,av]=pioneer_los(x,y,th,xstart,ystart,xend,yend);

N=41;
M=21;
xx = linspace(-100,100,N);
yy = linspace(-50,50,M);
[X,Y]=meshgrid(xx,yy);
Lv = [];
Av = [];
for ii = 1:size(X,1)
    for jj = 1:size(X,2)
        [d,Lv(ii,jj),Av(ii,jj)]=pioneer_los(X(ii,jj),Y(ii,jj),th,xstart,ystart,xend,yend);
    end
end

figure(1)
clf()
u = Lv.*cos(Av);
v = Lv.*sin(Av);
quiver(X,Y,u,v,0.5)
hold on
plot(xstart,ystart,'go','markerfacecolor','g','markersize',15)
plot(xend,yend,'ro','markerfacecolor','r','markersize',15)
axis('equal')
xlabel('X [m]')
ylabel('Y [m]')
