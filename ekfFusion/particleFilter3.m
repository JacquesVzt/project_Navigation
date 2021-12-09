function [x_hat,x_hat_mean,Xhat,Yhat,Xhat_mean,Yhat_mean] = particleFilter3(gnss_data_tt,imu_data_tt,ref_track_map)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%   x_hat=[x,y,vx,vy,ax,ay,d,v,a,theta,w]
b_disp=false;
dT=1;
dimx=11;
Tmax=2200;
N=500;
nb_oups=0;
nb_ok=0;
sigma_w= 0.05*10;
sigma_v=2;
sigma_xgnss=10;
sigma_acc = 0.005*9.81*100000000000000000;
time_gnss = seconds(gnss_data_tt.Time);
time_imu = seconds(imu_data_tt.Time);
sim_time = union(time_gnss,time_imu);
% sim_time = sim_time(sim_time<=sim_time(1)+Tend_i);
sim_steps = length(sim_time);
sim_steps=min(Tmax,sim_steps);
valid_gnss_indices = find(~isnan(gnss_data_tt.Latitude_deg));
[~,~,initial_utm_zone] = ll2utm(gnss_data_tt.Latitude_deg(valid_gnss_indices(1)),gnss_data_tt.Longitude_deg(valid_gnss_indices(1)),'wgs84');
[utm_x,utm_y,utm_zone_session_i] = ll2utm(gnss_data_tt.Latitude_deg,gnss_data_tt.Longitude_deg,'wgs84',initial_utm_zone);
x_hat=zeros(dimx,sim_steps,N); % size dimx * T * N
x_hat_mean=zeros(dimx,sim_steps);
Xhat=zeros(sim_steps,N);
Yhat=zeros(sim_steps,N);
Xhat_mean=0*(1:sim_steps);
Yhat_mean=0*(1:sim_steps);
t_i_last_ekf_call = 1;
x0=zeros(dimx,N);
D=ref_track_map.TrackDistance_m(end);
L=0:(D/N):D;
x0(7,:)=L(1:N);
L=rand(N,1)*5-2;
x0(11,:)=L;

for i=1:N
    [x,y,th,~]=curv(x0(7,i),ref_track_map);
    x0(1,i)=x;
    x0(2,i)=y;
    x0(10,i)=th;
    v=rand()*2-1;
    x0(3,i)=v*cosd(th);
    x0(4,i)=v*sind(th);
    a=rand()*2-1;
    x0(5,i)=a*cosd(th);
    x0(6,i)=a*sind(th);
end
% disp("x0="+num2str(x0))
W=ones(N,1)/N;
x_hat(:,1,:)=x0;
% for t_i=2:Tmax
for t_i=2:sim_steps
    s_gnss=sigma_xgnss*(1+rand()*0);
    logW=ones(N,1);
    logWacc=ones(N,1);logWw=ones(N,1);logWx=ones(N,1);
    sim_time_i = sim_time(t_i);
%     utm_zone_i = utm_zone_session_i(t_i);
    sim_time_i_last_ekf_call = sim_time(t_i_last_ekf_call);
    sample_time_i = sim_time_i-sim_time_i_last_ekf_call;
    sigma_x=gnss_data_tt.LatitudeSigma_m(t_i)*0+s_gnss;
    sigma_y=gnss_data_tt.LongitudeSigma_m(t_i)*0+s_gnss;
    [x_gnss,y_gnss,~]=ll2utm(gnss_data_tt.Latitude_deg(t_i),gnss_data_tt.Longitude_deg(t_i),'wgs84',initial_utm_zone);
    ax_imu=imu_data_tt.AccX_mss(t_i);
    ay_imu=imu_data_tt.AccY_mss(t_i);
    w_imu=imu_data_tt.TurnRateZ_degs(t_i);
    for i=1:N
        x0_i=zeros(dimx,1);
        x0_i(:)=x0(:,i);
        x_temp_i=x0_i;
        x_temp_i(7)=x0_i(7)+dT*x0_i(8);
        x_temp_i(8)=normrnd(x0_i(8),sigma_v);
        x_temp_i(9)=(x_temp_i(8)-x0_i(8))/dT;
        if x_temp_i(7)<0
            x_temp_i(7)=0;
            x_temp_i(8)=0;
        end
        x0(:,i)=x_temp_i;
        [x,y,th,b]=curv(x0_i(7),ref_track_map);
        x_temp_i(1)=x;
        x_temp_i(2)=y;
        x_temp_i(3)=(x_temp_i(1)-x0(1,i))/dT;
        x_temp_i(4)=(x_temp_i(2)-x0(2,i))/dT;
        x_temp_i(5)=(x_temp_i(3)-x0(3,i))/dT;
        x_temp_i(6)=(x_temp_i(4)-x0(4,i))/dT;
        x_temp_i(10)=th;
        x_temp_i(11)=(x_temp_i(10)-x0_i(10))/dT;
        x0(:,i)=x_temp_i;
        Xhat(t_i,i)=x;
        Yhat(t_i,i)=y;
        ax = x0(5,i);
        ay = x0(6,i);
        w=x0(11,i);
%         ax=imu_data_tt.
        Xhat_mean(t_i)=Xhat_mean(t_i)+x;
        Yhat_mean(t_i)=Yhat_mean(t_i)+y;

        if b
            W(i)=0;
            logW(i)=10^100;
            logWacc(i)=1;logWw(i)=1;logWx(i)=1;
        else
            if isnan(x_gnss) || isnan(y_gnss) 
                W(i)=W(i)*1;
                logW(i)=10^100;
                logWacc(i)=1;logWw(i)=1;logWx(i)=1;
            else
                W(i)=W(i)*exp(-(x_gnss-x)^2/2/sigma_x^2)*exp(-(y_gnss-y)^2/2/sigma_y^2)*exp(-(ax_imu-ax)^2/2/sigma_acc^2)*exp(-(ay_imu-ay)^2/2/sigma_acc^2)*exp(-(w_imu-w)^2/2/sigma_w^2);
                logW(i)=(x_gnss-x)^2/2/sigma_x^2+(y_gnss-y)^2/2/sigma_y^2+((ax_imu-ax)^2/2/sigma_acc^2+(ay_imu-ay)^2/2/sigma_acc^2)*1+(w_imu-w)^2/2/sigma_w^2*1;
                logWacc(i)=((ax_imu-ax)^2/2/sigma_acc^2+(ay_imu-ay)^2/2/sigma_acc^2);
                logWw(i)=(w_imu-w)^2/2/sigma_w^2;
                logWx(i)=(x_gnss-x)^2/2/sigma_x^2+(y_gnss-y)^2/2/sigma_y^2;
            end
        end
    end
    if sum(W)==0 

        nb_oups=nb_oups+1;
        disp("oups: "+num2str(nb_oups)+" "+num2str(nb_ok))
        try
            indice=randsample(1:N,N,true,1./logW);
            if min(logW)==0
                1
            end
        catch
            1
        end
        W=ones(N,1)/N;
    else
        nb_ok=nb_ok+1;
        W=W/sum(W);
        try
            indice=randsample(1:N,N,true,W);
        catch
            1
        end
        W=W(indice);
    end
    x0(:,:)=x0(:,indice);
    x_hat(:,t_i,:)=x0(:,:);
    x_=pagemtimes(squeeze(x_hat(:,t_i,:)),W);
    x_hat_mean(:,t_i)=x_;
    t_i_last_ekf_call=t_i;
end
Xhat_mean=Xhat_mean/N;
Yhat_mean=Yhat_mean/N;