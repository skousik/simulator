function make_plot_pretty(fh)
% make_plot_pretty()
% make_plot_pretty(fh)
%
% This sets things the way Shreyas likes 'em.
%
% Authors: Shreyas Kousik
% Created: 6 May 2021
% Updated: 18 Jan 2022
    if nargin < 1
        fh = gcf ;
    end

    set_plot_linewidths(1.5,fh)
    set_plot_fontsize(15,fh)
    
    % set marker sizes, woo!
    h = findobj(fh,'Marker','.') ;
    set(h,'MarkerSize',12) ;
end