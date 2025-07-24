function z=saturasi(x,lb,ub)
    if (x>ub)
        z=ub;
    elseif (x<lb)
        z=lb;
    else
        z=x;
    end
end