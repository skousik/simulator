function save_figure_to_png(figure_handle,filename)
% save_figure_to_png(figure_handle,filename)
%
% Save the figure associated with the figure_handle input to a PNG that
% minimizes the amount of whitespace around the figure.
%
% Code by "Friedrich" from MATLAB answers
% (https://www.mathworks.com/matlabcentral/answers/12987-how-to-save-a-matlab-graphic-in-a-right-size-pdf)
%
% Function created by Shreyas Kousik
% Created: 26 Aug 2020
% Updated: -

    % size the figure correctly
    set(figure_handle,'Units','Inches');
    pos = get(figure_handle,'Position');
    set(figure_handle,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    
    % save the figure
    print(figure_handle,filename,'-dpng','-r0')
end