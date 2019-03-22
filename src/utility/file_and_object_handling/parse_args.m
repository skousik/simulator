function obj = parse_args(obj,args_list)
    for idx = 1:2:length(args_list)
        obj.(args_list{idx}) = args_list{idx+1} ;
    end
end