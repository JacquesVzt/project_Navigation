gnss_data=eval(gnss_inatm200stn_processed_data_root_string);
session_i=2;
gnss_data_tt=gnss_data{session_i};
imu_data = eval(imu_inatm200stn_processed_data_root_string);
imu_data_tt=imu_data{session_i};
% ref_track_mapi = create_ref_track_map2(ref_ekfFusion_inatm200stn_data{session_i},utm_zone{session_i,1}.Data);
ref_track_mapi=ref_track_map;
Ts=[1];
if ~isempty(Ts)
    time_gnss = seconds(gnss_data_tt.Time);
    time_imu = seconds(imu_data_tt.Time);

    gnss_data_selector = [true;false(length(time_gnss)-1,1)];
    i_old = 1;
    for i = 2:length(time_gnss)
        ts_gnss_i = time_gnss(i)-time_gnss(i_old);
        if ts_gnss_i >= Ts
            i_old = i;
            gnss_data_selector(i-1) = true;
        end % if
    end % for i

    imu_data_selector = [true;false(length(time_imu)-1,1)];
    i_old = 1;
    for i = 2:length(time_imu)
        ts_imu_i = time_imu(i)-time_imu(i_old);
        if ts_imu_i >= Ts
            i_old = i;
            imu_data_selector(i-1) = true;
        end % if
    end % for i

    gnss_data_tt = gnss_data_tt(gnss_data_selector,:);
    imu_data_tt = imu_data_tt(imu_data_selector,:);
end % if

[x_hat,x_hat_mean,Xhat,Yhat,Xhat_mean,Yhat_mean] = particleFilter3(gnss_data_tt,imu_data_tt,ref_track_mapi);

n=length(ref_track_mapi.Latitude_deg);
m=length(gnss_data_tt.Latitude_deg);
Xtrack=0*(1:n);
Ytrack=0*(1:n);
Xgnss=0*(1:m);
Ygnss=0*(1:m);
for i=1:n
    [Xtrack(i),Ytrack(i)]=ll2utm(ref_track_mapi.Latitude_deg(i),ref_track_mapi.Longitude_deg(i));
end
for i=1:m
    [Xgnss(i),Ygnss(i)]=ll2utm(gnss_data_tt.Latitude_deg(i),gnss_data_tt.Longitude_deg(i));
end

printing