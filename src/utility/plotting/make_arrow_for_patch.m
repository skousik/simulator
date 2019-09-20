function [F,V] = make_arrow_for_patch(p1,p2,varargin)
    % set default values based on how far apart p1 and p2 are
    d = norm(p2 - p1) ;
    shaft_width = d/20 ;
    head_length = d/5 ;
    head_width = d/10 ;
    
    % iterate through varargin
    for idx = 1:2:length(varargin)
        switch varargin{idx}
            case 'shaft_width'
                shaft_width = varargin{idx+1} ;
            case 'head_length'
                head_length = varargin{idx+1} ;
            case 'head_width'
                head_width = varargin{idx+1} ;
            otherwise
                error('Invalid keyword!')
        end
    end

    % make the shaft
    shaft_length = d - head_length ;
    [F_shaft,V_shaft] = make_cylinder_for_patch(shaft_width/2,shaft_length) ;
    
    % make the head and shift it up
    [F_head,V_head] = make_cone_for_patch(head_length,head_width/2) ;
    V_head(:,3) = V_head(:,3) + shaft_length ;
    
    % stack all the vertices
    N_shaft = size(V_shaft,1) ;
    F_head = F_head + N_shaft ;
    F = [F_shaft ; F_head] ;
    V = [V_shaft ; V_head] ;
    
    % get the rotation matrix for the direction between p1 and p2
    n0 = [0;0;1] ;
    n = (p2 - p1)./d ;
    v = cross(n0,n) ;
    v_hat = skew(v) ;
    s = norm(v) ;
    c = dot(n0,n) ;
    R = eye(3) + v_hat + v_hat^2 .* ((1-c)./(s^2)) ;
    
    % rotate and translate the vertices
    V = (R*V')' + repmat(p1(:)',size(V,1),1) ;
end