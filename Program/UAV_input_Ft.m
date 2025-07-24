   function x=UAV_input_Ft(x,Ft,Ftp,dt)


% vx=x(4)+0.1*Ft(1);
% vy=x(5)+0.1*Ft(2);
% vz=x(6)+0.1*Ft(3);
vx=(Ft(1)-Ftp(1))/dt;
vy=(Ft(2)-Ftp(2))/dt;
vz=(Ft(3)-Ftp(3))/dt;
% vx=saturasi(vx,-2.5,2.5);
% vy=saturasi(vy,-2.5,2.5);
% vz=saturasi(vz,-2.5,2.5);

r=zeros(9,1);
r(1)=x(1)+Ft(1);
r(2)=x(2)+Ft(2);
r(3)=x(3)+Ft(3);
r(4)=vx;
r(5)=vy;
r(6)=vz;
dx=pers_state_uavnkontroller(x,r); %dinamika robot + kontroller
%     disp([a v]);
%     if (mod(i,100)==0)
%         r=[80;80;100].*rand(3,1)-[40;40;0];
%     end
%     disp([x(1:3) x(4:6) x(7:9)]);
x=x+dt*dx; % dinamika diskritisasi
end

