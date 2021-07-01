function [output] = saturate(value, satLim)
    if(value > satLim)
        output = satLim;
    elseif(value < -satLim)
        output = -satLim;
    else
        output = value;
    end
end

