function [list, oppositelist] = saveLetter(sequence, shift)
    % Initialize the oppositeList to be empty
    % 09/2025 F.G.Kyrgios

    % Check if the sequence matches type 1: A|A|b|b|C|C|a|a|B|B|c|c|
    if contains(sequence, '|')
        % Type 1: Split the string by the '|' separator
        list = strsplit(sequence, '|');
        % Remove any empty elements in case there's a trailing '|'
        list = list(~cellfun('isempty', list));
        
        % Reverse the list
        % list = flip(list);

        % Create oppositelist by shifting list elements
        shift = mod(shift, length(list)); % Ensure shift stays within bounds
        oppositelist = circshift(list, -shift); % Shift left by given amount
        % Swap b <-> c and B <-> C
        swapPairs = containers.Map({'b','c','B','C'}, {'c','b','C','B'});
        for k = 1:numel(list)
            if isKey(swapPairs, list{k})
                list{k} = swapPairs(list{k});
            end
        end
        for k = 1:numel(oppositelist)
            if isKey(swapPairs, oppositelist{k})
                oppositelist{k} = swapPairs(oppositelist{k});
            end
        end
        return;
    end
    
    % Check if the sequence matches type 2: A-b-C-a-B-c-A-b-C-a-B-c-
    if contains(sequence, '-')
        % Type 2: Create a list with letters and opposite case letters
        list = cellstr(num2cell(sequence(sequence ~= '-')));
        
        % Convert to opposite case and shift the letters in opposite order
        charList = num2cell(sequence(sequence ~= '-'));
        oppositeList = cell(1, length(charList));
        
        for i = 1:length(charList)
            if isstrprop(charList{i}, 'lower')  % If the character is lowercase
                oppositeList{i} = upper(charList{i});  % Convert to uppercase
            else
                oppositeList{i} = lower(charList{i});  % Convert to lowercase
            end
        end
        
        % Reverse the list
        % list = flip(list);
        % oppositeList = flip(oppositeList);

        % Shift by 1 to make the assignment right
        if ~isempty(oppositeList)
            oppositeList = [oppositeList(2:end), oppositeList(1)]; % shift the list
        end
        
        oppositelist = oppositeList;
        return;
    end
    
    % Check if the sequence matches type 3: AabBCcaABbcCAabBCcaABbcC
    if ischar(sequence) && all(isletter(sequence))
        % Type 3: Apply your original provided code for this type
        list = cellstr(num2cell(sequence));
        
        % Initialize the opposite case list as before
        charList = num2cell(sequence);
        oppositeList = cell(1, length(charList));
        
        % Loop over each character and convert to opposite case
        for i = 1:length(charList)
            if isstrprop(charList{i}, 'lower')  % If the character is lowercase
                oppositeList{i} = upper(charList{i});  % Convert to uppercase
            else
                oppositeList{i} = lower(charList{i});  % Convert to lowercase
            end
        end

        % Reverse the list
        % list = flip(list);
        % oppositeList = flip(oppositeList);
        
        if ~isempty(oppositeList)
            oppositeList = [oppositeList(2:end), oppositeList(1)];
        end
       
        oppositelist = oppositeList;
        return;
    end
    
    % Default case (if no pattern matched)
    list = {};
    oppositelist = {};
end

%{
% Test Type 1
[list, oppositelist] = saveLetters('A|A|b|b|C|C|a|a|B|B|c|c|', 0);
disp(list); % {'A', 'A', 'b', 'b', 'C', 'C', 'a', 'a', 'B', 'B', 'c', 'c'}
disp(oppositelist); % {}
disp("epomeno");

% Test Type 2
[list, oppositelist] = saveLetters('A-b-C-a-B-c-A-b-C-a-B-c-', 0);
disp(list); % {'A', 'b', 'C', 'a', 'B', 'c', 'A', 'b', 'C', 'a', 'B', 'c'}
disp(oppositelist); % {'a', 'B', 'c', 'A', 'b', 'C', 'a', 'B', 'c', 'A', 'b', 'C'}
disp("epomeno");
% Test Type 3
[list, oppositelist] = saveLetters('AabBCcaABbcC', 0);
disp(list); % {'A', 'a', 'b', 'B', 'C', 'c', 'a', 'A', 'B', 'b', 'c', 'C', ...}
disp(oppositelist); % {'a', 'A', 'B', 'b', 'c', 'C', 'A', 'b', 'a', 'B', 'C', ...}
%}