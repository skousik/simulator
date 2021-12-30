function [P,n] = make_random_convex_polygon(n,c)
% P = make_random_convex_polygon(n,c)
% [P,n] = make_random_convex_polygon(n,c)
%
% Make a random 2-D convex polygon with at most n sides, centered (roughly)
% at the point c \in \R^2
%
% The optional second output is the new number of *unique* vertices of the
% output; this may be less than the input n
%
% Usage example:
% P = make_random_convex_polygon(5)
% figure(1) ; clf ; grid on ; axis equal ;
% plot(P(1,:),P(2,:),'b')
%
% Authors: Shreyas Kousik
% Created: 8 Dec 2020
% Updated: 29 Dec 2021

    if nargin < 2
        c = zeros(2,1) ;
    elseif nargin < 1
        n = 3 ;
    end
    
    P = [] ;
    t_max = 1 ; % [seconds]
    t_start = tic ;
    
    while (size(P(:,1:end-1),2) < n) && (toc(t_start) <= t_max)
        P = make_random_polygon(n,c(:)) ;
        K = convhull(P') ;
        P = P(:,K) ;
    end
    
    if (size(P(:,1:end-1),2) < n)
        warning(['Failed to create polygon with required number of sides! ',...
            'Output has only ',num2str(size(P,2)),' sides, whereas ',...
            num2str(n),' sides were requested.'])
    end
    
    if nargout > 1
        n = size(P,2) - 1;
    end
end