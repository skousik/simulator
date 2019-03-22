function [F,V] = make_cuboid_for_patch(L,W,D)
    % using https://www.mathworks.com/help/matlab/visualize/multifaceted-patches.html
    
    % make vertices
    Vx = L.*[0 1 1 0 0 1 1 0]' - L/2 ;
    Vy = W.*[0 0 1 1 0 0 1 1]' - W/2 ;
    Vz = D.*[0 0 0 0 1 1 1 1]' - D/2 ;
    V = [Vx Vy Vz] ;
    
    % make faces
    F = [1 2 6 5 ; 2 3 7 6 ; 3 4 8 7 ; 4 1 5 8 ; 1 2 3 4 ; 5 6 7 8 ] ;
end