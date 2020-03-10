% [h, action] = selectfigure(name[, action])
%
%		Provides a way of selecting figures based on a text
%		string rather than on a number. The optional input
%		argument "action" is described below. If it is not
%		supplied, the default action chosen depends on the
%		conditions. If the figure does not exist, it is
%		"create". If it does, the action chosen is "raise" (if
%		no output argument was requested) or "get" (if at least
%		one output argument was requested). The second output
%		argument "action", if requested, returns the action
%		performed.
%
%   The above is the default behaviour if only one argument
%   is supplied. If the optional argument "action" is
%   included, the action performed is as following:
%
%   "create": create and make current (if the figure exists,
%		  it is destroyed first).
%
%   "raise": raise the figure (if the figure does not exist,
%		  it is created first).
%
%   "select": make the figure current (if the figure does
%     not exist, it is created first).
%
%   "get": return the figure handle (if the figure does not
%     exist, empty is returned).

% Author: Ben Mitch
% Version: 21/11/2010 (b)
%
% 21/11/2010 (b)
%		Fixed typo introduced at last update, 21/11/2010 (a).
%
% 28/02/2011
%   Introduced "action" parameter for more control.

% Copyright (c) 2010, Ben Mitch
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without 
% modification, are permitted provided that the following conditions are 
% met:
% 
%     * Redistributions of source code must retain the above copyright 
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright 
%       notice, this list of conditions and the following disclaimer in 
%       the documentation and/or other materials provided with the distribution
%       
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
% POSSIBILITY OF SUCH DAMAGE.



function [h, action] = selectfigure(name, action)



%% USAGE

if (nargin ~= 1 && nargin ~= 2) || ~ischar(name) || ndims(name) ~= 2 || size(name, 1) ~= 1 || size(name, 2) < 1
	disp(' ')
	help selectfigure
	error('invalid usage');
end



%% IMPLEMENTATION USING FIGURE NAME

% this is a nice implementation, since it automatically
% handles figure deletion by the user (a new figure will
% then be created).



%% GET EXISTING

h_fig = [];
c = get(0,'children');
for n = 1:length(c)
	
	% if name matches, retrieve it
	if strcmp(get(c(n), 'Name'), name)
		h_fig = c(n);
		break;
	end
	
end



%% INFER ACTION

% infer
if nargin < 2
	if isempty(h_fig)
		action = 'create';
	elseif nargout
		action = 'get';
	else
		action = 'raise';
	end
end

% subactions
sub_destroy = false;
sub_create = false;
sub_raise = false;
sub_select = false;
switch action
	case 'create'
		sub_destroy = true;
		sub_create = true;
		sub_raise = true;
	case 'raise'
		sub_create = true;
		sub_raise = true;
	case 'select'
		sub_create = true;
		sub_select = true;
	case 'get'
		% no sub actions
	otherwise
		error(['unrecognised action "' action '"']);
end



%% SUB: DESTROY

if sub_destroy && ~isempty(h_fig)
	close(h_fig);
	h_fig = [];
	drawnow
end



%% SUB: CREATE

if sub_create && isempty(h_fig)
	h_fig = figure( ...
		'Name', name, ...
		'NumberTitle', 'off', ...
		'IntegerHandle', 'off' ...
		);
end



%% SUB: RAISE

if sub_raise && ~isempty(h_fig)
	figure(h_fig);
end



%% SUB: SELECT

if sub_select && ~isempty(h_fig)
	set(0, 'CurrentFigure', h_fig);
end



%% SUB: GET

if nargout
	h = h_fig;
end



