function out = rand_in_bounds(bounds,m,s,n)
% random_point = rand_in_bounds(bounds,m,s,n)
%
% Create random points in the bounds [xmin xmax ymin ymax] (in 2-D) or in
% [xmin xmax ymin ymax zmin zmax] in 3-D. The second and third arguments
% are a mean and standard deviation, which are optional (if these are not
% passed in, this draws from a uniform distribution). The fourth argument
% is the number of points to return.
%
% To get n uniformly distributed points, set m = [] and s = [].
%
% The output random_point is either a 2-by-n or 3-by-n set of points in the
% provided bounds.
%
% Author: Shreyas Kousik
% Created: 27 Aug 2020
% Updated: 6 Jan 2021

    if nargin < 4
        n = 1 ;
    end

    if length(bounds) == 4
        lo = bounds([1,3]) ;
        hi = bounds([2,4]) ;
        n_dim = 2 ;
    elseif length(bounds) == 6
        lo = bounds([1,3,5]) ;
        hi = bounds([2,4,6]) ;
        n_dim = 3 ;
    else
        error(['Please pass in bounds as either [xmin xmax ymin ymax],'...
            ' or [xmin xmax ymin ymax zmin zmax]'])
    end
    
    if nargin < 2
        out = rand_range(lo(:),hi(:)) ;
    else
        out = rand_range(lo(:),hi(:),m,s,n_dim,n) ;
    end
end