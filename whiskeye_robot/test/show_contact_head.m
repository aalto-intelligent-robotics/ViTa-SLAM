
function show_contact_head

% start global node if not already
try
	rosinit
end

% prepare figure for display
selectfigure show_contact_head
clf
p = panel();
p.pack(2, 2);
q = {p(1, 1) p(1, 2) p(2, 1) p(2, 2)};

% panels
for i = 1:4
	
	q{i}.select();
	d = 0.3;

	switch i
		case 1
			h_cont = plot3(NaN, NaN, NaN, 'ro');
			xlabel('x')
			ylabel('y')
			zlabel('z')
			hold on
			plot3([-d/4 d], [0 0], [0 0], 'r-');
			plot3([0 0], [-d d], [0 0], 'g-');
			plot3([0 0], [0 0], [-d d], 'b-');
			view(30, 30)
		case 2
			h_norm = plot3(NaN, NaN, NaN, 'r.-');
			xlabel('x')
			ylabel('y')
			zlabel('z')
			hold on
			plot3([-d/4 d], [0 0], [0 0], 'r-');
			plot3([0 0], [-d d], [0 0], 'g-');
			plot3([0 0], [0 0], [-d d], 'b-');
			view(30, 30)
		case 3
			h_1 = plot(NaN, NaN, 'ro');
			xlabel('x')
			ylabel('y')
			hold on
			plot([-d/4 d], [0 0], 'r-');
			plot([0 0], [-d d], 'g-');
		case 4
			h_2 = plot(NaN, NaN, 'ro');
			xlabel('y')
			ylabel('z')
			hold on
			plot([-d d], [0 0], 'g-');
			plot([0 0], [-d d], 'b-');
	end
	
	axis equal

end

% subscribe
sub = rossubscriber('/whiskeye/head/contact_head', @contact_head_callback);

% collated
X = zeros(6, 0);

% receiver loop
while true

	% prep to display norms
	xyz = X(1:3, :); % contact location
	dxyz = X(4:6, :); % force vector
	scale = 100; % display scale
	Y = [xyz; xyz+dxyz*scale; NaN*X(1:3, :)];
	Y = reshape(Y, 3, []);
	
	% update figure
	set(h_cont, 'xdata', X(1, :), 'ydata', X(2, :), 'zdata', X(3, :));
	set(h_1, 'xdata', X(1, :), 'ydata', X(2, :));
	set(h_2, 'xdata', X(2, :), 'ydata', X(3, :));
	set(h_norm, 'xdata', Y(1, :), 'ydata', Y(2, :), 'zdata', Y(3, :));
	
	% yield
	pause(0.1)
	
end


	function contact_head_callback(a, b)
		
		reports = reshape(b.Data, 6, []);
		if size(reports, 2) == 0
			return
		end
		
		X = [X reports];
		
	end

end
