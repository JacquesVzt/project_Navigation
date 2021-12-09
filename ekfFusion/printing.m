figure(100)
hold on
beg=3;
end_=2000;
plot(Xtrack,Ytrack, 'k-')

N=100;
for i =1:5
%     plot(Xhat(beg:end_,i),Yhat(beg:end_,i),'c.')
end
% plot(Xgnss(2:end),Ygnss(2:end), 'g.-')
% plot(Xhat(3:end,i),Yhat(3:end,i),'ro-')

% plot(Xhat_mean(beg:end_),Yhat_mean(beg:end_), 'b.-')

% plot(Xtrack,Ytrack, 'b.-')

figure(104)
hold on
for i =1:5
    plot(Xhat(beg:end_,i),'c.')
end
plot(Xgnss(2:end),'g.-')
% plot(Xhat(3:end,i),'ro-')
plot(Xhat_mean(3:end_), 'b.-')
% hold off
% plot(Xhat(3:end,i),'kx-')

figure(105)
hold on
for i =1:5
    plot(Yhat(beg:end_,i),'c.')
end
plot(Ygnss(2:end),'g.-')
% plot(Yhat(3:end,i),'ro-')

plot(Yhat_mean(beg:end_), 'b.-')
% hold off
% plot(Yhat(3:end,i),'kx-')
