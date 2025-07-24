 function y = saturate(x, lower, upper)
        if x > upper
            y = upper;
        elseif x < lower
            y = lower;
        else
            y = x;
        end
    end
    