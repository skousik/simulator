function varargout = make_sphere(r,c,n)
% S = make_sphere(r,c,n)
% [F,V] = make_sphere(r,c,n)
% [S1,S2,S3] = make_sphere(r,c,n)
%
% Make a 3-D sphere of radius r, centered at the point c, containing
% approximately n points.
%
% All inputs are optional. The defaults are:
%   r = 1, c = zeros(3,1), n = 100
%
% The number of outputs is optional:
%   S is a 3-by-n (approximately) array of points on the sphere
%   [F,V] is a faces/vertices pair to be used with patch
%   [S1,S2,S3] are matrices to be used with surf or mesh
%
% Authors: Shreyas Kousik
% Created: 15 Mar 2021
% Updated: not yet

if nargin < 1
    r = 1 ;
end

if nargin < 2
    c = zeros(3,1) ;
end

if nargin < 3
    n = 100 ;
end

% get n for sphere
n_sphere = floor(sqrt(n-1)) ;

% create sphere
[X_1,X_2,X_3] = sphere(n_sphere) ;

% use radius and center
X_1 = r.*X_1 + c(1) ;
X_2 = r.*X_2 + c(2) ;
X_3 = r.*X_3 + c(3) ;

% create output
switch nargout
    case 1
        X = [X_1(:), X_2(:), X_3(:)]' ;
        varargout = {X} ;
    case 2
        [F,V] = surf2patch(X_1,X_2,X_3) ;
        varargout = {F,V} ;
    case 3
        varargout = {X_1,X_2,X_3} ;
end