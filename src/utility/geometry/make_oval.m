function [O,F] = make_oval(s,N)
% B = make_box(s)
% [B,F] = make_box(s)
% [B,F] = make_box(s,N)
%
% Return a 2-by-M oval of xy-coordinates that are CCW that fits in a box of
% side length s (if s is a scalar) or s(1)-by-s(2) (if s is a vector). The
% optional second input N determines the number of sides to use to
% approximate the circular ends of the oval.

    if nargin < 2
        N = 20 ;
    end

    if length(s) == 1
        % if s is a scalar, return a circle
        O = make_circle(s,N) ;
    else
        % if s is a vector, first sort s
        if s(1) > s(2)
            s = s([2 1]) ;
        end
        
        r = s(1)/2 ; % radius of caps
        l = s(2) - 2*r ; % length between cap centers
        
        % create the left cap
        t_left = linspace(pi/2,3*pi/2,ceil(N/2)) ;
        x_left = r*cos(t_left) - l/2 ;
        y_left = r*sin(t_left) ;
        
        % create the right cap
        t_right = linspace(-pi/2,pi/2,ceil(N/2)) ;
        x_right = r*cos(t_right) + l/2 ;
        y_right = r*sin(t_right) ;
        
        % create the oval
        O = [x_left, x_right ; y_left, y_right] ;
    end
    
    if nargout > 1
        F = [1:length(O), 1] ;
    else
        O = [O, O(:,1)] ;
    end
end