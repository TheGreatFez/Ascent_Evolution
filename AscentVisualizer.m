X = linspace(0,70000,1000)';
pitch = 90;
switch_alt = 5000;
for iX =1:length(X)
    if X(iX) > switch_alt && pitch > 0
        switch_alt = switch_alt + 5000;
        pitch = pitch - 15;
    end
    Y(iX,1) = pitch;    
end

plot(X,Y)
grid on
xlabel('Altitude (m)')
ylabel('Pitch Above Horizon (deg)')
title('Pitch vs Altitude')