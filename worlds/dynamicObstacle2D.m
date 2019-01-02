classdef dynamicObstacle2D < obstacle2D

methods
%% constructor
function O = dynamicObstacle2D(vertices,position,time)
    O@obstacle2D(vertices) ;
    O.position = position ;
    O.time = time ;
    O.type = 'dynamic' ;
end

end
end