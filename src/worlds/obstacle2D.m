classdef obstacle2D < handle
properties
    type % 'static' or 'dynamic'
    vertices
    position
    time
end

methods
%% constructor
function O = obstacle2D(vertices,position,time)
    % Static:  O = obstacle2D(vertices)              
    % Dynamic: O = obstacle2D(vertices,position,time)
    %
    % Create an obstacle2D instance with the given vertices. If time and
    % position vectors are supplied as well, then the obstacle will be
    % dynamic.
    %
    % If no inputs are provided, the obstacle is created as a random static
    % obstacle with 3 to 9 vertices, centered at (0,0).
    
    if nargin < 2
        position = [] ;
        time = [] ;
        type = 'static' ;
        
        if nargin < 1
            vertices = makeRandomPolygon() ;
        end
    else
        type = 'dynamic' ;
    end
    
    O.type = type ;
    O.vertices = vertices ;
    O.position = position ;
    O.time = time ;
end

%% utility
function vdisp(O,s,l)
% Display a string s if the message's verbose level l is greater
% than or equal to the planner's verbose level.
    if nargin < 3
        l = 1 ;
    end
    if O.verbose >= l
        if ischar(s)
            disp(['        O: ',s])
        else
            disp('        O: String not provided!')
        end
    end
end
end
end