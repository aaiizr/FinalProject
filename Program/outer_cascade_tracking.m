function [r7,r8]=outer_cascade_tracking(x,r)
    s7=(x(1)-r(1))*cos(x(9))+(x(2)-r(2))*sin(x(9));
    s8=-(x(1)-r(1))*sin(x(9))+(x(2)-r(2))*cos(x(9));
    v7=(x(4)-r(4))*cos(x(9))+(x(5)-r(5))*sin(x(9));
    v8=-(x(4)-r(4))*sin(x(9))+(x(5)-r(5))*cos(x(9));
    s=sqrt(s7*s7+s8*s8);
    v=sqrt(v7*v7+v8*v8);
%     disp([arah7 arah8]);
%     disp([s7 s8 v7 v8]);
    sa=25;
    sb=6;
    vc=4;
    if (s>sa)
        a=1;
        b=0;
    elseif (s>sb)
        a=(s-sb)/(sa-sb);
        b=1-a;
    else
        a=0;
        b=s/sb;
    end
    if (v>vc)
        c=1;
    else
        c=v/vc;
    end
    rra=0.5;
    rrb=0.3;
    rrc=0.5;
    if(v>0 && s>0)
        r7=s8/s*rra*a+s8/s*rrb*b*(1-c)+v8/v*rrc*c*(1-a);
        r8=-(s7/s*rra*a+s7/s*rrb*b*(1-c)+v7/v*rrc*c*(1-a));
    elseif (s>0)
        r7=s8/s*rra*a+s8/s*rrb*b*(1-c);
        r8=-(s7/s*rra*a+s7/s*rrb*b*(1-c));
    elseif (v>0)
        r7=v8/v*rrc*c*(1-a);
        r8=-(v7/v*rrc*c*(1-a));
    else
        r7=0;
        r8=0;
    end
    if (r7~=0 && r8~=0)
        rrr=sqrt(r7^2+r8^2);
        if (rrr>0.5)
            r7=0.5*r7/rrr;
            r8=0.5*r8/rrr;
        end
    end
end
