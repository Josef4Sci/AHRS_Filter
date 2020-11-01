function mlapp2classdef(pathToMLapp, varargin)
% MLAPP2CLASSDEF converts an App Designer GUI's class definition, packaged 
% as a *.mlapp file, from XML to a standalone *.m class definition.
%
% MLAPP2CLASSDEF() prompts the user to select a single *.mlapp file for
% processing
%
% MLAPP2CLASSDEF(pathToMLapp) processes the files specified by the user.
% pathToMLapp can be a string for a single file or a cell array of strings
% for multiple files. Filepaths should be absolute.
%
% MLAPP2CLASSDEF(..., 'ReplaceAppUI', flag) replaces App Designer UI
% elements with their "regular" MATLAB equivalents (e.g. App Designer uses
% UIFIGURE where MATLAB uses FIGURE). flag is a boolean value, the default
% is false. To prompt the user to select an app file with this syntax, pass
% an empty first argument (e.g. MLAPP2CLASSDEF([], 'ReplaceAppUI', True)).
%
% The class definition for an App Designer GUI is embedded in an XML file
% located in a subfolder of the packaged *.mlapp file, which can be
% accessed like a *.zip file. MLAPP2CLASSDEF strips the XML header & footer
% and saves the class definition to a *.m file located in the same path as
% the *.mlapp file.
%
% MLAPP2CLASSDEF assumes that the targeted *.mlapp file is a GUI created by
% MATLAB's App Designer. Other packaged apps are not explicitly supported.
%
% GUIs converted utilizing this method will likely require R2014b or newer 
% to support addressing UI object properties using dot notation

if verLessThan('matlab', '8.2')
    error('mlapp2classdef:UnsupportedMATLABver', ...
          'MATLAB releases prior to R2013b are not supported' ...
          );
end

% Choose appropriate behavior based on number of inputs
if nargin == 0 || ~exist('pathToMLapp', 'var') || isempty(pathToMLapp)
    % No input selected, prompt user to select a MATLAB app to process
    % Currently limited to single file selection
    [filename, pathname] = uigetfile('*.mlapp', 'Select MATLAB App', 'MultiSelect', 'on');
    pathToMLapp = fullfile(pathname, filename);
    if ~filename
        error('mlapp2classdef:NoFileSelected', 'No file selected, exiting...');
    else
        % uigetfile's multiselect doesn't currently allow to select files
        % in different directories, so we can inherit the pathname(s) from
        % the uigetfile call.
        [~, appname, ext] = fileparts(filename);
    end
else
    % Wrap validateattributes for more verbose error handling
    % validateattributes won't catch if the cell array contains
    % non-strings, but the subsequent fileparts call will error if these
    % are encountered
    pathToMLapp = validateattributes_wrapped(pathToMLapp, {'char', 'cell', 'string'}, {'vector'});
    if iscell(pathToMLapp)
        [pathname, appname, ext] = cellfun(@fileparts, pathToMLapp, 'UniformOutput', false);
    else
        [pathname, appname, ext] = fileparts(pathToMLapp);
    end
    filename = strcat(appname, ext);
end

optionflags = checkflags(varargin);

if iscell(pathToMLapp)
    for indF = 1:numel(pathToMLapp)
        checkfile(pathname{indF}, filename{indF}, ext{indF});
        processapp(pathname{indF}, filename{indF}, appname{indF}, optionflags)
        % TODO: Add a counter of successfully converted files.
    end
else
    checkfile(pathname, filename, ext);
    processapp(pathname, filename, appname, optionflags)
end

end


function A = validateattributes_wrapped(A, classes, attributes)
% Wrap validateattributes with try-catch block for more verbose error
% handling
try 
    validateattributes(A, classes, attributes)
catch err
    switch err.identifier
        case 'MATLAB:invalidType'
            newerr.identifier = 'mlapp2classdef:InvalidInputType';
            newerr.message = sprintf('Invalid input type: %s\nExpected: char, cell', class(A));
            newerr.cause = err.cause;
            newerr.stack = err.stack;
            error(newerr);
        case 'MATLAB:expectedVector'
            % Warn and reshape
            sizestr = sprintf('%u,', size(A));
            sizestr = sizestr(1:end-1);  % Strip trailing comma
            warning('mlapp2classdef:InvalidInputShape', ...
                    'Input cell array must be a vector of cells. Size of input array is: [%s]. Reshaping...', ...
                    sizestr ...
                    );
            A = reshape(A, 1, []);
        otherwise
            rethrow err
    end
end
end


function [optionflags] = checkflags(inputargs)
% Check main function varargin for optional processing flags
% Output a structure of flags with their values
if isempty(inputargs)
    % If no flags are input, utilize defaults
    optionflags.ReplaceAppUI = false;
else
    p = inputParser();
    p.FunctionName = 'mlapp2classdef';  % Throw errors as mlapp2classdef
    p.KeepUnmatched = true;  % Keeps unmatched N-V pairs and suppresses the error
    
    % Add our NV pairs
    addParameter(p, 'ReplaceAppUI', false, @islogical)
    
    % Parse function inputs and return results
    parse(p, inputargs{:});
    optionflags = p.Results;
end
end


function checkfile(pathname, filename, ext)
% Check for existence of file
if exist(fullfile(pathname, filename), 'file')
    % Check for correct file type
    if ~strcmp(ext, '.mlapp')
        error('mlapp2classdef:InvalidFileType', ...
            '''%s'' is not a *.mlapp file', fullfile(pathname, filename) ...
            );
    end
else
    error('mlapp2classdef:FileNotFound', ...
        '''%s'' does not exist', fullfile(pathname, filename) ...
        );
end
end


function processapp(pathname, filename, appname, uielementflag)
isolderthanR2014b = verLessThan('matlab', '8.4');
if isolderthanR2014b
    % *.mlapp is a *.zip file, extract the contents and strip out the XML
    % from the classdef
    tmpdir = unpackapp(pathname, filename, appname);
    rawXML = loadXML(tmpdir);
    mymcode = stripXML(rawXML);
    rmdir(tmpdir, 's');
else
    % Beginning with R2014b MATLAB's type function supports *.mlapp files,
    % so we can pipe the output to an external file rather then extract
    % from the *.zip file
    evalcstr = sprintf('type(''%s'')', fullfile(pathname, filename));
    mymcode = evalc(evalcstr);
    % Test for and strip out any leading whitespace characters
    if isspace(mymcode(1))
        mymcode(1) = [];
    end
    % Convert to cell array for compatibility with other routines
    mymcode = strsplit(mymcode, '\n', 'CollapseDelimiters', 0)';
end

if uielementflag.ReplaceAppUI
    % Convert App Designer UI elements to "regular" MATLAB UI elements
    regexdict = genregexdict();
    
    % As a starting point the UI elements will be addressed on an
    % individual basis.
    functionstoswap = fieldnames(regexdict);
    for ii = 1:length(functionstoswap)
        expression = regexdict.(functionstoswap{ii}).expression;
        replace = regexdict.(functionstoswap{ii}).replace;
        mymcode = regexprep(mymcode, expression, replace);
    end
    
    % Convert property declarations to backwards-compatible format
    mymcode = fixpropertydef(mymcode);
end

writemfile(mymcode, pathname, appname);
disp(strcat({'Successfully converted '}, filename, '!'));
end


function [tmpdir] = unpackapp(pathname, filename, appname)
% Unzip user selected MATLAB App, which are packaged in a renamed zip file
tmpdir = fullfile(pathname, sprintf('%s_tmp', appname));
unzip(fullfile(pathname, filename), tmpdir);
end


function [rawXML] = loadXML(tmpdir)
% Read in XML file
% Since there isn't really much XML-ness to this XML file, no need to
% utilize a full-fledged parser. MATLAB's won't open it anyway...
xmlfile = fullfile(tmpdir, 'matlab', 'document.xml');

% Get a count of lines in the xml file to preallocate the cell array in
% memory. If no count can be made, revert to growing the array in memory
nlines = countlines(xmlfile);
if ~isempty(nlines)
    rawXML = cell(nlines, 1);
else
    rawXML = {};
end

% Read XML file line-by-line into a cell array to make later export simpler
fID = fopen(xmlfile, 'r');
ii = 1;
while ~feof(fID)
    rawXML{ii} = fgetl(fID);
    ii = ii + 1;
end
fclose(fID);
end


function [mymcode] = stripXML(rawXML)
% Strip out XML header & footer
% Limit search to first & last lines of file, currently all that is
% modified by MATLAB to wrap the class definition in XML
mymcode = rawXML;
mymcode([1,end]) = regexprep(mymcode([1,end]), '(^.*)\[(?=classdef)|(?<=end)(\].*$)', '');
end


function [mymcode] = fixpropertydef(mymcode)
% Convert property type specifications to backwards compatible format
%
% Starting in R2016a, the documented method to declare class property types
% is to specify them after the property declaration with a space in between
% (e.g. HeaderLength uint16). This is different from the previous
% (undocumented) approach of separating them with the @ symbol (e.g.
% HeaderLength@uint16). While the old undocumented approach continues to
% function in R2016a, the documented R2016a convention is not backwards
% compatible.
%
% See: http://undocumentedmatlab.com/blog/setting-class-property-types-2
% for additional information

% Find where the property blocks start and end
propblockstart = find(~cellfun('isempty', regexp(mymcode, '^\s*properties', 'start')));
endstatements = find(~cellfun('isempty', regexp(mymcode, '^\s*end', 'start')));
if ~isempty(propblockstart)
    % We have at least one property block
    % Pair property block(s) with their end statement(s). Assumes that
    % the end statement following each property closes the property
    % block, so any logic control inside the property block will be
    % broken
    npropblocks = length(propblockstart);
    propertyblockpair = zeros(npropblocks, 2);
    for ii = 1:npropblocks
        propertyblockpair(ii, 1) = propblockstart(ii);
        % Find first end statement after the property block declaration
        propertyblockpair(ii, 2) = endstatements(find(endstatements > propblockstart(ii), 1));
        
        % Go through the block and swap the class property type syntax
        tmpblock = mymcode(propertyblockpair(ii,1) + 1:propertyblockpair(ii,2) - 1);
        tmpblock = regexprep(tmpblock, '^(\s*\w+)(\s+)(?![\%])', '$1\@');
        mymcode(propertyblockpair(ii,1) + 1:propertyblockpair(ii,2) - 1) = tmpblock;
    end
end
end


function writemfile(mymcode, pathname, appname)
% Write our m code to a file.
fID = fopen(fullfile(pathname, sprintf('%s.m', appname)), 'w');

% Write a cell array of strings to a *.m file
% Assumes each cell is a separate line
for ii = 1:length(mymcode)
    fprintf(fID, '%s\n', mymcode{ii});
end
fclose(fID);
end


function [nlines] = countlines(filepath)
% Count the number of lines present in the specified file.
% filepath should be an absolute path
fID = fopen(filepath, 'rt');

nlines = 0;
while ~feof(fID)
    nlines = nlines + sum(fread(fID, 16384, 'char') == char(10));
end

fclose(fID);
end


function [regexdict] = genregexdict()
% Build structure of regular expressions to swap function calls

% Replace uifigure with figure, make no changes to function inputs
regexdict.figureObj.expression = 'uifigure';
regexdict.figureObj.replace    = 'figure';

% Replace uiaxes with axes, make no changes to function inputs
regexdict.axesObj.expression = 'uiaxes';
regexdict.axesObj.replace    = 'axes';

% Replace uibutton with pushbutton uicontrol, assume only UIfunction input 
% is the parent object
regexdict.pushbuttonObj.expression = '(uibutton)\((.*)\)';
regexdict.pushbuttonObj.replace    = 'uicontrol(''Parent'', $2, ''Style'', ''pushbutton'')';

% Replace uicheckbox with checkbox uicontrol, assume only UIfunction input 
% is the parent object
regexdict.checkboxObj.expression = '(uicheckbox)\((.*)\)';
regexdict.checkboxObj.replace    = 'uicontrol(''Parent'', $2, ''Style'', ''checkbox'')';

% Replace uieditfield with edit uicontrol, assume only UIfunction input is 
% the parent object
regexdict.editboxObj.expression = '(uieditfield)\((.*)\)';
regexdict.editboxObj.replace    = 'uicontrol(''Parent'', $2, ''Style'', ''edit'')';

% Replace uilabel with text uicontrol, assume only UIfunction input is the
% parent object
regexdict.textObj.expression = '(uilabel)\((.*)\)';
regexdict.textObj.replace    = 'uicontrol(''Parent'', $2, ''Style'', ''text'')';

% Replace uilistbox with listbox uicontrol, assume only UIfunction input
% is the parent object
regexdict.listboxObj.expression = '(uilistbox)\((.*)\)';
regexdict.listboxObj.replace    = 'uicontrol(''Parent'', $2, ''Style'', ''listbox'')';

% Replace uiradiobutton with radiobutton uicontrol, assume only UIfunction 
% input is the parent object
regexdict.radiobuttonObj.expression = '(uiradiobutton)\((.*)\)';
regexdict.radiobuttonObj.replace    = 'uicontrol(''Parent'', $2, ''Style'', ''radiobutton'')';

% Replace uislider with slider uicontrol, assume only UIfunction input is
% the parent object
regexdict.sliderObj.expression = '(uislider)\((.*)\)';
regexdict.sliderObj.replace   = 'uicontrol(''Parent'', $2, ''Style'', ''slider'')';

% Replace uitogglebutton with pushbutton uicontrol, assume only UIfunction 
% input is the parent object
regexdict.togglebuttonObj.expression = '(uitogglebutton)\((.*)\)';
regexdict.togglebuttonObj.replace    = 'uicontrol(''Parent'', $2, ''Style'', ''togglebutton'')';

% Replace Value Changed callbacks with callback definition for "regular"
% graphics objects
regexdict.ValueChangedCallback.expression = '(\w+).ValueChangedFcn = createCallbackFcn\(\w+\, (.+)\)';
regexdict.ValueChangedCallback.replace    = '$1.Callback = {$2}';

% Replace Selection Changed callbacks with callback definition for 
% "regular" graphics objects
regexdict.SelectionChangedCallback.expression = '(\w+).SelectionChangedFcn = createCallbackFcn\(\w+\, (.+)\)';
regexdict.SelectionChangedCallback.replace    = '$1.Callback = {$2}';

% Remove App Designer registration
regexdict.registerAppFcn.expression =  '(registerApp.*)';
regexdict.registerAppFcn.replace    = '% Function call removed';
end
