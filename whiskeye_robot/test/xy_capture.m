
% CTRL-C to end
sub = rossubscriber('/whiskeye/head/xy');
msg = {};
while(1)
	msg{end+1} = receive(sub);
end
