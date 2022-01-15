function set_plot_markersize(markersize,fh)
% set_plot_markersize(fontsize,fh)
%
% Set all the marker sizes in the current figure, or else in the provided
% figure handle. The default size is 9.
%
% See also: set_plot_fontsize, set_plot_linewidth
%
% Authors: Shreyas Kousik
% Created: 12 Jan 2022
% Updated: noap

    if nargin < 1
        markersize = 15 ;
    end
    
    if nargin < 2
        fh = gcf ;
    end
    
    % send it
    set(findall(fh,'-property','MarkerSize'),'MarkerSize',markersize)
end