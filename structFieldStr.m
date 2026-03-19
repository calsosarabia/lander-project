function str = structFieldStr(struct)
str = '';

str = [str, sprintf('')];
fields = fieldnames(struct);

for i = 1:numel(fields)
    value = struct.(fields{i});
    
    if isnumeric(value) && isscalar(value)
        str = [str, sprintf('%s = %.6g, ', ...
            fields{i}, value)];
    else
        str = [str, sprintf('%s = %s\n', ...
            fields{i}, mat2str(value))];
    end
end

end