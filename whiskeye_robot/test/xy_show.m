
% collate
x = [];
N = length(msg);
for n = 1:N
	x(n, :) = msg{n}.Data;
end

% figure
selectfigure xy_show
clf
p = panel();
p.pack(4, 6);

% for each
i = 1;
for r = 1:6
	for c = 1:4
		p(c, r).select();
		plot(x(:, i+[0 1]));
		axis([0 N [-1 1]*1]);
		i = i + 2;
	end
end

