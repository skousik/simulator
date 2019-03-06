classdef waypointFinder2D < highLevelPlanner2D
% Class: waypointFinder2D
%
% This is a coarse path planner for planar autonomous robots. Given start
% and goal points, and obstacles defined as polygons, the waypoint finder 
% searches for a collision-avoiding path through the world.
%
% Required inputs to constructor function:
%   s                start position in R^2
%   g                desired goal position in R^2
%
% Varargin keywords:
%   gridSpacing      spacing between grid points (scalar)
%
%   gridType         'rect', 'hex', or 'rand' (string)
%
%	costIncrease     amount to increase grid edge cost for gridpoints 
%                    inside of obstacles (scalar)
%
%   costDecrease     amount to decrease grid edge cost for gridpoints that
%                    used to be in obstacles, but no longer are (useful for
%                    sensor returns that contain false positives) (scalar)
%
%   costMax          maximum cost of a single grid edge (scalar)
%
%   gridBounds       global bounds on grid; should enclose start and goal
%                    points to make sure a solution always exists
%                    (1-by-4 vector: [xlo xhi ylo yhi]); these are used to
%                    crop the final grid, regardless of its size or
%                    rotation, and by default are [-Inf Inf -Inf Inf]
%
%   gridSize         2-D size of grid (1-by-2 vector)
%
%   penaltyBuffer    amount to buffer obstacles by (scalar)
%
%   pathDiscount     discount applied to the edges of the current shortest
%                    path, which helps stabilize the A* algorithm's 
%                    solution from query to query as the robot moves 
%                    through the world (scalar)
%
%   rotateGrid       whether or not to rotate the grid (boolean)
%
%   rotationAngle    angle by which to rotate the grid; by default, this
%                    angle is chosen to align the start and goal points on
%                    the same axis (scalar)
%
%   gridConnectivity number of edges per grid point; 8 corresponds to the
%                    regular A* algorithm (integer)
%
% Generic Usage:
%   WP = waypointFinder2D(s,g,varargin)
%   WP.updateGraphWeights(WP,obstacles_as_CCW_polygons)
%   WP.getWaypoints(WP)
%   waypoint = WP.makeWaypoint(WP,lookahead_distance)
%
% Written by: Shreyas Kousik
% Latest version: 5 Jan 2018

    properties
	% user-entered properties:
        z % current position of vehicle
        g % global desired goal
        d % grid spacing
        b % buffer distance around obstacles to penalize points
        P % obstacles as CCW contours separated by NaNs
        gridtype % type of grid (rectangular, hexagonal, or random)
        cup % cost increase for grid points in obstacles (this should be large)
        cdn % cost decrease for grid points not in obstacles (this should be about 1/5 of cup)
        cmax % max cost of any point in the grid (this should be about 100*cup)
        pdn % decrease of cost along existing path (this should be >= d)
        N % number of points to connect to other grid points
    
    % auto-generated and maintained properties
        Z % state space grid
        l % number of points in Z
        A % adjacency matrix
        w % list of waypoints
        wdist % cumulative distance along path given by w
        znode % node closest to z
        gnode % node closest to g
        hzg % rotation of grid
    end
    
    methods
    %% constructor
        function WP = waypointFinder2D(s,g,varargin)
        % include the user inputs into the waypoint finder
            WP.z = s ; WP.g = g ;
            dzg = sqrt(sum((s(1:2) - g).^2,1)) ;
            
            if nargin > 2
                for vidx = 1:2:length(varargin)-1
                    switch lower(varargin{vidx})
                        case 'gridspacing'
                            d = varargin{vidx+1} ;
                        case 'gridtype'
                            gridtype = varargin{vidx+1} ;
                        case 'costincrease'
                            cup = varargin{vidx+1} ;
                        case 'costdecrease'
                            cdn = varargin{vidx+1} ;
                        case 'costmax'
                            cmax = varargin{vidx+1} ;
                        case 'gridbounds'
                            gridBounds = varargin{vidx+1} ;
                        case 'gridsize'
                            gridSize = varargin{vidx+1} ;
                        case 'penaltybuffer'
                            b = varargin{vidx+1} ;
                        case 'pathdiscount'
                            pdn = varargin{vidx+1} ;
                        case 'rotategrid'
                            rotate_grid = varargin{vidx+1} ;
                        case 'rotationangle'
                            hzg = varargin{vidx+1} ;
                        case 'gridconnectivity'
                            N = varargin{vidx+1} ;
                        otherwise
                            error(['Invalid option! Please read the latest ',...
                                  'help page for waypointFinder2D to make ',...
                                  'sure you are using the right keywords.'])
                    end
                end
            end
            
            if ~exist('d','var')
                d = dzg/16 ;
                if d == 0
                    d = 0.25 ;
                end
            end
            
            if ~exist('b','var')
                b = d/4 ;
            end
            
            if ~exist('gridtype','var')
                gridtype = 'rect' ;
            end

            if ~exist('cup','var')
                cup = 10*d ;
            end

            if ~exist('cdn','var')
                cdn = 5*d ;
            end

            if ~exist('cmax','var')
                cmax = 10e6 ;
            end
            
            if ~exist('pdn','var')
                pdn = d/2 ;
            end
            
            if ~exist('rotate_grid','var')
                rotate_grid = true ;
            end
            
            if ~exist('N','var')
                if strcmp(gridtype,'rect')
                    N = 8 ;
                else
                    N = 6 ;
                end
            end

            WP.d = d ;
            WP.b = b ; WP.gridtype = gridtype ;
            WP.cup = cup ; WP.cdn = cdn ; WP.cmax = cmax ;
            WP.pdn = pdn ;
            WP.N = N ;
            
        % create the state space grid
            % find the angle between z and g, so that the grid can be
            % rotated such that there is a straight line from z to g
            % initially available
            if ~exist('hzg','var')
                hzg = atan2(g(2)-s(2),g(1)-s(1)) ;
            end
            
            WP.hzg = hzg ;
            
        % create the grid area
            if ~exist('gridSize','var')
                % make sure the grid surrounds z and g by expanding it by
                % 8*d in all directions, and making sure it is long enough
                % in the x direction by adding dzg; the whole grid may be
                % rotated so that the x direction is pointing directly from
                % z to g
                xlo = -dzg/2 - 8*d ;
                xhi = +dzg/2 + 8*d ;
                ylo = -8*d ;
                yhi = +8*d ;
            else
                xlo = -gridSize(1)/2 ; xhi = gridSize(1)/2 ;
                ylo = -gridSize(2)/2 ; yhi = gridSize(2)/2 ;
            end
                
            if strcmp(WP.gridtype,'hex')
                % source: https://matlabdatamining.blogspot.com/2008/04/
                % generating-hexagonal-grids-for-fun-and.html
                xlo = xlo/(sqrt(3)/2) ; xhi = xhi/(sqrt(3)/2) ;
                ylo = ylo/(sqrt(3)/2) ; yhi = yhi/(sqrt(3)/2) ;
                
                xvec = xlo:d:xhi ;
                yvec = ylo:d:yhi ;
                [X,Y] = meshgrid(xvec,yvec) ;
                
                Y = (sqrt(3)/2).*Y ;
                [r,c] = size(Y) ;
                Xtemp = repmat([0, d/2]', [ceil(r/2), c]) ;
                if size(Xtemp,1) > r
                    Xtemp = Xtemp(1:end-1,:) ;
                end
                X = X + Xtemp ;
            elseif strcmp(WP.gridtype,'rand')
                Nx = ceil(abs(xhi - xlo)/d) ;
                Ny = ceil(abs(yhi - ylo)/d) ;
                Z = rand(2,Nx*Ny) ;
                X = (xhi-xlo).*Z(1,:) + xlo ;
                Y = (yhi-ylo).*Z(2,:) + ylo ;
            else
                xvec = xlo:d:xhi ;
                yvec = ylo:d:yhi ;
                [X,Y] = meshgrid(xvec,yvec) ;
            end
            
            if rotate_grid
                % rotate the grid to have z pointing at g initially
                R = [cos(hzg) -sin(hzg) ; sin(hzg) cos(hzg)] ;
            else
                R = eye(2) ;
            end
            
            % shift the grid by dzg/2 along its local x-axis, then rotate
            % it and shift it to z, which will position the center of the
            % grid on the center of the line between z and g
            Ztemp = [X(:) Y(:)]' ;
            Ztemp = Ztemp + repmat([dzg/2;0],1,size(Ztemp,2)) ;
            Ztemp = R*Ztemp + repmat(s(1:2),1,size(Ztemp,2)) ;
            
            % crop the grid by the input gridbounds
            if exist('gridBounds','var')
                xcroplo = gridBounds(1) ;
                xcrophi = gridBounds(2) ;
                ycroplo = gridBounds(3) ;
                ycrophi = gridBounds(4) ;
                
                Zcroplog = Ztemp(1,:) > xcroplo & Ztemp(1,:) < xcrophi & ...
                           Ztemp(2,:) > ycroplo & Ztemp(2,:) < ycrophi ;
                Ztemp = Ztemp(:,Zcroplog) ;
            end
                           
            WP.Z = Ztemp ;
            WP.l = size(WP.Z,2) ;
            
        % create the adjacency matrix
            WP.A = createAdjacencyMatrix(WP.Z,WP.N,WP.d) ;
        
        % create an empty obstacle, waypoints, and distance placeholders
            WP.P = [] ;
            WP.w = [] ;
            WP.wdist = [] ;
            
        % get znode and gnode
            [~,WP.znode] = min(sqrt(sum((WP.Z-repmat(WP.z(1:2),1,WP.l)).^2,1))) ;
            [~,WP.gnode] = min(sqrt(sum((WP.Z-repmat(WP.g,1,WP.l)).^2,1))) ;
        end
    
        
        
        
        
        
    %% update graph edge weights
        function  updateGraphWeights(WP,P)
            WP.P = P ;
            Amat = WP.A ;
            
            % check which points are in or near obstacles
            if ~isempty(P)
                [in,nr] = findPointsInOrNearObstacles(WP.Z, WP.P, WP.b) ;
            else
                in = [] ;
                nr = [] ;
            end
            
            if any(in)
                % create matrix of rows and columns corresponding to points
                % that are in obstacles
                In = repmat(in,length(in),1) ;
                In = In | In' ;
                
                % increase weight of all edges that are connected to
                % vertices in or near any obstacle
                Wmat = Amat.*In > 0 ;
                Amat = Amat + WP.cup.*Wmat ;

                % for any vertices that are over the max possible cost, reset them
                Amat(Amat > WP.cmax) = WP.cmax ;
                
                % for any vertices that are no longer in an obstacle,
                % reduce their cost
                Wmat = Amat.*not(In) > 0 & Amat.*not(In) > WP.cdn ;
                Amat = Amat - WP.cdn.*Wmat ;
                
                % put A back into WP
                WP.A = Amat ;
            end
            
            if any(nr)
                % create matrix of rows and columns corresponding to points
                % that are near obstacles
                Nr = repmat(nr,length(in),1) ;
                Nr = Nr | Nr' ;
                
                % increase weight of all edges that are connected to
                % vertices  near any obstacle
                Wmat = Amat.*Nr > 0 ;
                Amat = Amat + 0.5*WP.cup.*Wmat ;

                % for any vertices that are over the max possible cost, reset them
                Amat(Amat > WP.cmax) = WP.cmax ;
                
                % for any vertices that are not near an obstacle,
                % reduce their cost
                Wmat = Amat.*not(Nr) > 0 & Amat.*not(Nr) > WP.cdn ;
                Amat = Amat - 0.5*WP.cdn.*Wmat ;
                
                % put A back into WP
                WP.A = Amat ;
            end
        end
        
        
        
        
        
        
    %% generate path of waypoints
        function getWaypoints(WP,getCumulativeDist)
            if nargin < 2
                getCumulativeDist = false ;
            end
            
            % find node closest to z and between z and gw, but exclude the
            % half plane that is
%             htest = (0.5)*(WP.hzg+hcur) ;
%             zprev = WP.z ;
%             Zx = WP.Z(1,:) ; Zy = WP.Z(2,:) ;
%             Zlog = 1.*((cos(htest).*(Zx - zprev(1)) + sin(htest).*(Zy - zprev(2))) >= 0) ;
%             Zlog(~Zlog) = NaN ;
%             
%             [~,WP.znode] = min(sqrt(sum((WP.Z-repmat(WP.z(1:2),1,WP.l)).^2,1)) + Zlog) ;
            
            [~,WP.znode] = min(sqrt(sum((WP.Z-repmat(WP.z(1:2),1,WP.l)).^2,1))) ;
            
            % find node closest to g
            [~,WP.gnode] = min(sqrt(sum((WP.Z-repmat(WP.g,1,WP.l)).^2,1))) ;
            
            % run shortest path algorithm
            [~,path,~] = graphshortestpath(WP.A,WP.znode,WP.gnode) ;
                       
            % return waypoints along path
            wtemp = WP.Z(:,path) ;
            WP.w = wtemp ;
            
            if getCumulativeDist
                wcumdist = cumulativeDist(wtemp') ;
                WP.wdist = wcumdist' ;
            end
            
            % decrease edge weights along existing path
            pbeg = path(1:end-1) ; pfin = path(2:end) ;
            Amat = WP.A ;
            [r,c] = size(Amat) ;
            Bmat = sparse(r,c) ;
            Bmat(pbeg,pfin) = 1 ;
            Alog = Amat.*Bmat > WP.pdn ;
            Amat = Amat - Alog.*WP.pdn ;

            %% this was left unfinished!
%             % increase weight of edges that have large heading changes
%             % first get heading of each segment
%             h = atan2(diff(wtemp(2,:)),diff(wtemp(1,:))) ;
%             
%             % absolute difference in heading between adjacent segments
%             dh = [abs(diff(h)),0] ;
%             disp(h)
%             disp(dh)
%             Bmat([ind1,ind2]) = (2/norm(dh)).*WP.cup.*[dh,dh] ;
%             Amat = Amat + Bmat ;
            
%             % increase weight of edges that intersect obstacles
%             [~,~,in] = polyxpoly(WP.w(1,:)',WP.w(2,:)',...
%                                  WP.P(1,:)',WP.P(2,:)') ;
%             if ~isempty(in)
%                 % the first column of 'in' is the indices of the /edges/ in
%                 % WP.w that intersect with obstacles, which are given by
%                 % the starting vertex; these need to be translated from
%                 % indices of the waypoints to indices of the graph vertices
%                 wbeg = in(:,1) ; % (starting indices of intersecting edges
%                                  % in the waypoints)
%                 pwbeg = path(wbeg) ;
%                 pwfin = path(wbeg + 1) ;
%                 ind1 = sub2ind([r,c],pwbeg,pwfin) ;
%                 ind2 = sub2ind([r,c],pwfin,pwbeg) ;
%                 
%                 Wmat = sparse(r,c) ;
%                 Wmat([ind1,ind2]) = 1 ;
%                 
%                 Amat = Amat + Wmat.*WP.cup ;
%                 disp('edges intersect obstacles!')
%             end
         
            
            WP.A = Amat ;
        end
        
        
        
        
        
        
	%% output waypoint along the path
        function [wp,h] = makeWaypoint(WP,ddes)
            % Given a desired distance along the path of waypoints, output
            % a waypoint wp that is interpolated from the path
            
            % get distance along polyline that the vehicle is currently at
            wps = WP.w ;
            dw = distAlongPolyline(WP.z,wps) ;
            cw = WP.wdist ; % cumulative distance along waypoints
            
            D = dw + ddes ;
            
            % ddes is the distance ahead of the vehicle to look... if this
            % exceeds the cumulative possible distance, then the waypoint
            % is simply the last point of the waypoint path
            if size(wps,2) <= 1 || D > cw(end)
                wp = WP.g ;

                if nargout > 1
                    % get the heading from the current position to the goal and
                    % maintain it
                    h = atan2(WP.g(2) - WP.z(2), WP.g(1) - WP.z(1)) ;
                else
                    h = [] ;
                end
            else
                wpx = interp1(cw,WP.w(1,:),D) ;
                wpy = interp1(cw,WP.w(2,:),D) ;
                wp = [wpx;wpy] ;

                if nargout > 1
                    if (D + 0.01) < cw(end)
                        wp_next_x = interp1(cw,wps(1,:), D + 0.01) ;
                        wp_next_y = interp1(cw,wps(2,:), D + 0.01) ;
                    else
                        wp_next_x = wps(1,end) ;
                        wp_next_y = wps(2,end) ;
                    end
                    
                    if (D - 0.01) > 0
                        wp_prev_x = interp1(cw,wps(1,:), D - 0.01) ;
                        wp_prev_y = interp1(cw,wps(2,:), D - 0.01) ;
                    else
                        wp_prev_x = wps(1,1) ;
                        wp_prev_y = wps(2,1) ;
                    end
                    
                    if (wp_prev_x == wp_next_x && wp_prev_y == wp_next_y) || ...
                       any(isnan([wp_prev_x,wp_prev_y,wp_next_x,wp_next_y]))
                        h = atan2(WP.g(2) - WP.z(2), WP.g(1) - WP.z(1)) ;
                    else
                        dx = wp_next_x - wp_prev_x ;
                        dy = wp_next_y - wp_prev_y ;
                        h = atan2(dy,dx) ;
                    end
                else
                    h = [] ;
                end
            end
            
%             cumd = WP.wdist ;
%             if size(WP.w,2) > 1 && ddes < cumd(end)

%             
%                 if getHeading
%                     if ddes < (cumd(end)-0.02)
%                         % this is an if statement in case the waypoint needs to
%                         % interpolate beyond the available waypoint path
%                         wp_next_x = interp1(cumd,WP.w(1,:),ddes+0.01) ;
%                         wp_next_y = interp1(cumd,WP.w(2,:),ddes+0.01) ;
%                     else
%                         wp_next_x = WP.g(1) ;
%                         wp_next_y = WP.g(2) ;
%                     end
% 
%                     wp_prev_x = interp1(cumd,WP.w(1,:),ddes-0.01) ;
%                     wp_prev_y = interp1(cumd,WP.w(2,:),ddes-0.01) ;
% 
%                     % find heading to that waypoint and have it be desired
%                     dx = wp_next_x - wp_prev_x ;
%                     dy = wp_next_y - wp_prev_y ;
% 
%                     h = atan2(dy,dx) ;
%                     % disp(['Desired heading: ',num2str(h)])
%                 else
%                     h = [] ;
%                 end
%             else
%                 wp = WP.g ;
%                 
%                 % get the heading from the current position to the goal and
%                 % maintain it
%                 h = atan2(WP.g(2) - WP.z(2), WP.g(1) - WP.z(1)) ;
%             end
        end
    end
end






%% create adjacency matrix
function A = createAdjacencyMatrix(XY,N,d)
    % first use KNN to get the N nearest neighbors of each point
    [idx,D] = knnsearch(XY',XY','K',N+1) ;

    % for points without N neighbors, eliminate the extra points by
    % eliminating the corresponding indices from KNN
    %% TO DO: fix this
    dmax = (sqrt(N)/2).*sqrt(2).*d ;
    idx = idx .* (D <= dmax) ;

    l = size(XY,2) ;
    A = zeros(l,l) ;

    for xidx = 1:size(idx,1)
        % get the adjacent nodes (i.e. the distance isn't 0)
        yidxlog = idx(xidx,:) ~= 0 ;
        yidx = idx(xidx,yidxlog) ;

        % if not intersecting, put the corresponding distances into the
        % adjacency matrix
        didx = abs((D(xidx,yidxlog)).^(0.9)) ; % changed this to a weird power 15 Nov 17 2:10PM
        A(xidx,yidx) = didx ;
        A(yidx,xidx) = didx ;
    end

    A = sparse(A) ;
end






%% find points in or near obstacles
function [in, nr] = findPointsInOrNearObstacles(Z,P,d)
    if nargin < 3
        d = 0 ;
    end
    
    % find points inside obstacles
    in = inpolygon(Z(1,:)',Z(2,:)',P(1,:)',P(2,:)') ;
    
    % find points near obstacles
    [~,D] = knnsearch(P',Z') ;
    nr = D < d ;
    
    if ~isempty(in)
        in = sparse(in(:)') ;
    end
    
    if ~isempty(nr)
        nr = sparse(nr(:)') ;
    end
end
