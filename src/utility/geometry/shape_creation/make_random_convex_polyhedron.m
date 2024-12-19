function [F,V] = make_random_convex_polyhedron(N,c)
% Given a number of vertices N and a center c \in \R^3, return a structure
% S that has two fields (Faces and Vertices) that can be used as the input
% to the patch function, or return [F,V] as the faces and vertices
% matrices.
%
% Author: Shreyas Kousik
% Created: 5 July 2024
% Updated: --

    if nargin < 1
        N = round(rand_range(4,12)) ;
    end
    
    if nargin < 2
        c = zeros(3,1) ;
    end
    
    c = c(:)' ;
    
    % make vertices
    V = (2.*rand(N,3)-1) + repmat(c,N,1) ;
    
    % get convex hull
    F = convhull(V) ;
    
    % discard vertices that are not in convex hull
    V = V(unique(F(:)),:) ;
    
    % get convex hull again hehe
    F = convhull(V) ;
    
    % create output
    if nargout == 1
        S.faces = F ;
        S.vertices = V ;
        F = S ;
    end
end