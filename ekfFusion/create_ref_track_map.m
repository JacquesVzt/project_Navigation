function ref_track_map2 = create_ref_track_map(ref_ekfFusion_inatm200stn_data,utm_zone)
T=length(ref_ekfFusion_inatm200stn_data.AccX_mss);
TrackDistance_m=[];
Longitude_deg=[];
Latitude_deg=[];
Heading_deg=[];
d=0;
intd=0;
T
[x0,y0]=ll2utm(ref_ekfFusion_inatm200stn_data.Latitude_deg(1),ref_ekfFusion_inatm200stn_data.Longitude_deg(1));
[x1,y1]=ll2utm(ref_ekfFusion_inatm200stn_data.Latitude_deg(2),ref_ekfFusion_inatm200stn_data.Longitude_deg(2));
th0=90-(atan2d(y1-y0,x1-x0));
if th0>180
    th0=th0-360;
end
if th0<-180
    th0=th0+360;
end

X=zeros(1,T);
Y=zeros(1,T);
for t_i=1:T
    [X(t_i),Y(t_i)]=ll2utm(ref_ekfFusion_inatm200stn_data.Latitude_deg(t_i),ref_ekfFusion_inatm200stn_data.Longitude_deg(t_i));
end
% lconv=10;
% X=[repmat(X(1),1,lconv),X,repmat(X(end),1,lconv)];
% X=conv(X,ones(1,lconv+1)/(lconv+1),'same');
% X=X((1+lconv):(end-lconv));
% Y=[repmat(Y(1),1,lconv),Y,repmat(Y(end),1,lconv)];
% Y=conv(Y,ones(1,lconv+1)/(lconv+1),'same');
% Y=Y((1+lconv):(end-lconv));
x_=x0;
y_=y0;
X0=[];
Y0=[];
valid=true;
for t_i=1:T
%     disp("t:"+num2str(t_i))
%     disp("x_:"+num2str(x_))
%     disp("x0:"+num2str(x0))
    x=X(t_i);
    y=Y(t_i);
%     [x,y]=ll2utm(ref_ekfFusion_inatm200stn_data.Latitude_deg(t_i),ref_ekfFusion_inatm200stn_data.Longitude_deg(t_i));
    l=sqrt((x-x0)^2+(y-y0)^2)^0.5;
    if d+l>=intd
        th=90-(atan2d(y-y0,x-x0));
        dth=th-th0;
        if dth>350
            dth=dth-360;
        elseif dth<-350
            dth=dth+360;
        end
        [La,Lo]=utm2ll(x,y,utm_zone(1));
        X0(intd+1)=x;
        Y0(intd+1)=y;
        Longitude_deg(intd+1)=Lo;%ref_ekfFusion_inatm200stn_data.Longitude_deg(t_i);
        Latitude_deg(intd+1)=La;%ref_ekfFusion_inatm200stn_data.Latitude_deg(t_i);
        TrackDistance_m(intd+1)=intd;
        Heading_deg(intd+1)=90-(atan2d(y-y0,x-x0));
        if Heading_deg(intd+1)>180
            Heading_deg(intd+1)=Heading_deg(intd+1)-360;
        end
        if Heading_deg(intd+1)<-180
            Heading_deg(intd+1)=Heading_deg(intd+1)+360;
        end
        intd=intd+1;
    end
%     disp("x:"+num2str(x))
    d=d+l;
    if valid
        x_=x0;y_=y0;
        x0=x;y0=y;th0=th;
    end
    valid=true;
end
Heading_deg(1)=Heading_deg(2);

ref_track_map2=table(TrackDistance_m,Longitude_deg,Latitude_deg,Heading_deg);
