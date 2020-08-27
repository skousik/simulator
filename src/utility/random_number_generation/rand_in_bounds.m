function out = rand_in_bounds(bounds,m,s)
% random_point = rand_in_bounds(bounds,m,s)
%
% Create a random point in the bounds [xmin xmax ymin ymax] (in 2-D) or in
% [xmin xmax ymin ymax zmin zmax] in 3-D. The second and third arguments
% are a mean and standard deviation, which are optional (if these are not
% passed in, this draws from a uniform distribution).
%
% The output random_point is either a 2-by-1 or 3-by-1 point in the bounds.
%
% Author: Shreyas Kousik
% Created: 27 Aug 2020
    if length(bounds) == 4
        lo = bounds([1,3]) ;
        hi = bounds([2,4]) ;
    elseif length(bounds) == 6
        lo = bounds([1,3,5]) ;
        hi = bounds([2,4,6]) ;
    else
        error(['Please pass in bounds as either [xmin xmax ymin ymax],'...
            ' or [xmin xmax ymin ymax zmin zmax]'])
    end
    
    if nargin < 2
        out = rand_range(lo(:),hi(:)) ;
    else
        out = rand_range(lo(:),hi(:),m,s) ;
    end
end