function u = make_random_unit_vector(D)
    u = 2.*rand(D,1) - 1 ;
    u = u./norm(u) ;
end