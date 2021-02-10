function X = make_grid(bounds,n_per_dim)
% X = make_grid(bounds,n_per_dim)
%
% Make an m-D grid of points in the bounds [x_1_lo x_1_hi ... x_n_lo x_m_hi]
% where there are n_per_dim = [n_1 n_2 ... n_m] points along each dim. The
% output is an m-by-prod(n_per_dim) array of points.
%
% See also: make_grid_2D
%
% Authors: Shreyas Kousik
% Created: 9 Feb 2021
% Updated: nah

if nargin < 2
    n_per_dim = [100 100] ;
end

if nargin < 1
    bounds = [-1 1 -1 1] ;e
end

% get number of generators
n_dim = size(bounds,2)/2 ;

% reshape the bounds
bounds = reshape(bounds(:),2,n_dim) ;

% generate grid of points
x_vec_in = {} ;
for idx = 1:n_dim
    x_vec_in{idx} = linspace(bounds(1,idx),bounds(2,idx),n_per_dim(idx)) ;
end

cell_out = cell(1,n_dim) ;
[cell_out{:}] = ndgrid(x_vec_in{:}) ;
X = [] ;
for idx = 1:length(cell_out)
    X = [X ; cell_out{idx}(:)'] ;
end