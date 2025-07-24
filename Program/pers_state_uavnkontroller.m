function dx=pers_state_uavnkontroller(x,r)
    dx=[0;0;0;0;0;0;0;0;0;0;0;0]; 
    %3 posisi x y z
    %3 kecepatan translasi x y z
    %3 orientasi roll pitch yaw
    %3 kecepatan sudut roll pitch yaw
    %parameter
    mm=1;
    g=10;
    Ix=0.03;
    Iy=0.03;
    Iz=0.05;
    dd=0.7; %gaya gesek udara/drag
    d=3.13e-3; %koefisien gerak yaw
    L=0.2; %koefisien gerak roll/pitch
    
    %Parameter Kontroller
    K2=100;
    L2=21;
    K3=K2;
    L3=L2;
    K4=0.09;
    L4=0.61;
    K1=20;
    L1=9;
    %Kontroller Ketinggian dulu aja deh
    %xr= [1 2 3 4    5    6    7     8     9     10 11 12]
    %x = [x y z u    v    w    roll  pitch yaw   p  q  r]
    %r = [x y z xdot ydot zdot xddot yddot zddot]
    u1=mm*(g+K1*(r(3)-x(3))+L1*(r(6)-x(6))+r(9)+dd/mm*r(6))/cos(x(7))/cos(x(8));
    u1=saturasi(u1,0,20); %
    Peb=[1 sin(x(7))*tan(x(8)) cos(x(7))*tan(x(8))
        0 cos(x(7)) -sin(x(7))
        0 sin(x(7))/cos(x(8)) cos(x(7))/cos(x(8))];
    dx(1:3)=x(4:6);
    dx(4)=(sin(x(9))*sin(x(7))+cos(x(9))*cos(x(7))*sin(x(8)))*u1/mm-dd*x(4)/mm;
    dx(5)=(-cos(x(9))*sin(x(7))+sin(x(9))*cos(x(7))*sin(x(8)))*u1/mm-dd*x(5)/mm;
    dx(6)=-g+cos(x(7))*cos(x(8))*u1/mm-dd*x(6)/mm;
%     dx(7)=x(10);
%     dx(8)=x(11);
%     dx(9)=x(12);
    dx(7:9)=Peb*x(10:12);
    
    %Kontroller orientasi
    s8=sin(x(8))*cos(x(7));
    s7=sin(x(7));
%     r7=0.4; r8=0.4;
    
    
    ds8=dx(8)*cos(x(7))*cos(x(8))-dx(7)*sin(x(7))*sin(x(8));
    ds7=dx(7)*cos(x(7));
    
    if (x(9)>0.1 || x(9)<-0.1)
        u2=Iy/L*(-K2*s7-L2*ds7);
        u3=Ix/L*(-K3*s8-L3*ds8);
    else
        [r7,r8]=outer_cascade_tracking(x,r);
%         disp([r7 r8]);
        u2=Iy/L*(K2*r7-K2*s7-L2*ds7);
        u3=Ix/L*(K3*r8-K3*s8-L3*ds8);
    end
    u4=Iz/d*(-K4*x(9)-L4*x(12));
%     disp([u1 u2 u3 u4]);
    %untuk pdot, qdot, rdot sama dengan teori di buku tesis
    dx(10)=((Iy-Iz)*x(11)*x(12)+u2*L)/Ix;
    dx(11)=((Iz-Ix)*x(10)*x(12)+u3*L)/Iy;
    dx(12)=((Ix-Iy)*x(10)*x(11)+u4*d)/Iz;
end

