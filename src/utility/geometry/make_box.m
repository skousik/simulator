function [B,F] = make_box(s)
% B = make_box(s)
% [B,F] = make_box(s)
%
% Return a 2-by-5 "box" of xy-coordinates that are CCW with side length
% given by s(1) (for a square) or s = [s1, s2] for a rectangle
%
% The second (optional) output is a vector F that contains the order of the
% vertices in B used to define a patch face: F = [1 2 3 4 1]. In this case,
% the box B is returned as a 2-by-4 list of vertices.
%
% Examples:
%   makeBox() returns a square of side length 2
%
%   makeBox(2) returns a box of side length 4
%
%   makeBox([1 2]) returns a 1x2 m box

    if nargin < 1
        L = 1 ; W = 1 ;
    elseif length(s) == 1
        L = s ; W = s ;
    else
        L = s(1) ; W = s(2) ;
    end

    x = (L/2)*[-1 1 1 -1 -1] ;
    y = (W/2)*[-1 -1 1 1 -1] ;

    B = [x;y] ;

    if nargout > 1
        B = B(:,1:4) ;
        F = [1 2 3 4 1] ;
    end
end