figure('Color', 'w');
clf
c = colormap(lines(3));


A = randn(60,7);        % some data
A(:,4) = NaN;           % this is the trick for boxplot
C = [c; ones(1,3); c];  % this is the trick for coloring the boxes


% regular plot
boxplot(A, 'colors', C, ...
    'labels', {'','ASIA','','','','USA',''}); % label only two categories
hold on;
for ii = 1:3
    plot(NaN,1,'color', c(ii,:), 'LineWidth', 4);
end

title('BOXPLOT');
ylabel('MPG');
xlabel('ORIGIN');
legend({'SUV', 'SEDAN', 'SPORT'});

set(gca, 'XLim', [0 8], 'YLim', [-5 5]);