X = linspace(0,70000,1000)';
switch_alt = 221;
end_alt    = 70000;
alt_diff = end_alt - switch_alt;
scaleFactor = 1:-0.1:0.5;
legendName = {};
close all
for iScale =1:length(scaleFactor)
    end_alt = 70000*scaleFactor(iScale);
    alt_diff = end_alt - switch_alt;
    for iX =1:length(X)
        Y(iX,1) = max(0,min(90,90*sqrt((X(iX) - switch_alt)/alt_diff)));
    end
    figure(1)
    hold on
    plot(X,Y)
    grid on
    xlabel('Altitude (m)')
    ylabel('Pitch Above Horizon (deg)')
    title('Pitch vs Altitude')
    axis([0 70000 0 100])
    legendName{iScale} = ['Scale Factor = ' num2str(scaleFactor(iScale))];

end

legend(legendName,'Location','southeast')
