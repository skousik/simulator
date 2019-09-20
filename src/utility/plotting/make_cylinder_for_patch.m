function [F,V] = make_cylinder_for_patch(radius,height,N)
    if nargin < 3
        N = 10 ;
    end
    
    % create points
    [x,y,z] = cylinder([radius, radius],N) ;
    
    % make z the height and center the cylinder at the origin
    z(2,:) = height ;
    
    % make patch
    [F,V] = surf2patch(x,y,z) ;
end