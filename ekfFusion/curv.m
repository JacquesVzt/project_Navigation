function [x,y,th,b]=curv(d,ref_track_map)
D=ref_track_map.TrackDistance_m;
if d>D(end)
%     disp("d>D(end)")
    [x,y]=ll2utm(ref_track_map.Latitude_deg(end),ref_track_map.Longitude_deg(end));
    th=ref_track_map.Heading_deg(end);
    b=true;
else
    i=find(D>=d,1);
    if i==1
%         disp("i==1,D(end)="+num2str(D(end))+", d="+num2str(d))
        [x,y]=ll2utm(ref_track_map.Latitude_deg(1),ref_track_map.Longitude_deg(1));
        th=ref_track_map.Heading_deg(1);
        b=true;
    else
%         disp("d="+num2str(d))
        [xa,ya]=ll2utm(ref_track_map.Latitude_deg(i-1),ref_track_map.Longitude_deg(i-1),'wgs84');
        tha=ref_track_map.Heading_deg(i-1);
        da=D(i-1);
        [xb,yb]=ll2utm(ref_track_map.Latitude_deg(i),ref_track_map.Longitude_deg(i),'wgs84');
        thb=ref_track_map.Heading_deg(i);
        db=D(i);
        L=[xa,ya,tha]*(db-d)/(db-da)+[xb,yb,thb]*(d-da)/(db-da);
        x=L(1);
        y=L(2);
        th=L(3);
        b=false;
    end
end
