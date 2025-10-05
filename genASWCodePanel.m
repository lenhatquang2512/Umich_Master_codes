% /****************************************************************************
%  * Auto Generate C code from ASW Models
%  * Copyright (C) 2025
%  *
%  * @author Quang Nhat Le - v.quangln7@vinfast.vn
%  * @date   2025-5-9
%  ****************************************************************************/
clc; clear all mex; close all; format default;
% genASWCodePanel_();
runGenASWCodePanelWithDir();

function genASWCodePanel_()

    clc; clear all mex; close all; format default;
    
    % Auto-detect available SWC_*.slx files in the current folder
    swcFiles = dir('SWC_*.slx');     % Look for SWC_*.slx in current directory

    % Also look in known subfolders
    modelFolders = {'BCC','CSPF','HSPF','MCF','MDF','MPC','RCF','SCF','TDC','PCF','TPC','VCF'};
    for i = 1:length(modelFolders)
        if isfolder(modelFolders{i})
            % addpath(genpath(modelFolders{i})); % add folder + subfolders to path
            subFiles = dir(fullfile(modelFolders{i}, 'SWC_*.slx'));
            swcFiles = [swcFiles; subFiles]; %concatenating
        end
    end

    % If none found,log errors
    if isempty(swcFiles)
        errordlg('No SWC_*.slx models found in the current folder or model subfolders.', 'Error');
        return;
    end

    % Collect model names (relative to where found)
    modelNames = cellfun(@(f) erase(f, '.slx'), {swcFiles.name}, 'UniformOutput', false);

    % Create GUI figure
    f = figure('Name','VinFast EDS ASW CodeGen Panel', 'Position',[500 400 350 250], ...
               'MenuBar','none', 'NumberTitle','off', 'Resize','off');
    % f = figure('Name', 'VinFast EDS ASW CodeGen Panel', ...
    %        'Position', [500 400 350 250], ...
    %        'MenuBar', 'none', ...
    %        'NumberTitle', 'off', ...
    %        'Resize', 'off', ...
    %        'WindowStyle', 'modal');  % makes figure always on top and blocks other windows

    % Add popup menu
    uicontrol('Style','text', 'Position',[75 190 200 20], ...
              'String','Select an ASW model to import:', 'FontWeight','bold');
    popup = uicontrol('Style','popupmenu', 'Position',[75 160 200 25], ...
                      'String', modelNames);

    % Add checkbox for code generation
    zipCheckbox = uicontrol('Style', 'checkbox', ...
                                'String', 'Zip code delivery after generating', ...
                                'Position', [90 125 250 20], ...
                                'Value', 1);  % Checked by default

    % Add Generate button
    uicontrol('Style','pushbutton', 'String','Generate Code', ...
              'Position',[125 80 100 30], ...
              'Callback', @(src,event)generateCBCodeNormalASWModel(popup, modelNames, f, zipCheckbox));

    % === Add Progress Bar (initially empty) ===
    ax = axes('Parent', f, 'Units','pixels', ...
              'Position',[50 30 250 20], ...
              'XTick',[], 'YTick',[], 'Box','on', ...
              'Tag','ProgressBar');
    xlim(ax,[0 1]); ylim(ax,[0 1]);
    title(ax, 'Progress: 0%', 'FontSize',8, 'Color','k');

    % coffee time
    uicontrol('Style','text', ...
          'Position',[55 5 250 20], ...   % adjust Y position as needed
          'String','Please buy me a coffee - quangln7', ...
          'FontWeight','bold', 'ForegroundColor',[0.5 0.2 0]);

end

function generateCBCodeNormalASWModel(popup, modelNames, figHandle, zipCheckbox)

    % Get selected index and model name
    idx = get(popup, 'Value');
    modelName = modelNames{idx};

    %Check if user selected code generation
    doZip = get(zipCheckbox, 'Value');

    %Check matlab ver
    matlabVer = checkMatlab();

    totalSteps = 5;   % define total steps of workflow
    step = 1;

    % Close figure
    % close(figHandle);

    % --- Detect where the model file lives ---
    modelFile = [modelName '.slx'];
    modelFolder = '';  % will store the folder if model is inside subdir

    if isfile(modelFile)
        % model is in current folder
        modelFolder = pwd;
    else

        % check known subfolders
        searchFolders = {'BCC','CSPF','HSPF','MCF','MDF','MPC','RCF','SCF','TDC','PCF','TPC','VCF'};
        found = false;

        for i = 1:numel(searchFolders)
            candidate = fullfile(searchFolders{i}, modelFile);
            if isfile(candidate)
                modelFolder = fullfile(pwd, searchFolders{i});
                modelFile   = candidate;
                found = true;
                break;
            end
        end

        if ~found
            errordlg(['Model file not found: ' modelFile], 'Error');
            return;
        end

    end

    % --- Switch to model folder and add it to path ---
    fprintf('Switching to model folder: %s\n', modelFolder);
    cd(modelFolder);
    addpath(modelFolder);
    
    % Step 2 - Create progress bar initially
    updateProgressBar(figHandle, step, totalSteps, 'Preparing model...');
    
    % Remove slprj if it exists
    step = step + 1;
    updateProgressBar(figHandle, step, totalSteps, 'Cleaning workspace...');
    if isfolder('slprj')
        try
            rmdir('slprj', 's');  % 's' for recursive delete
            fprintf('Deleted existing "slprj" folder to avoid conflicts.\n');
        catch ME
            warning(['Could not delete "slprj" folder: ' getReport(ME, 'basic')]);
        end
    end
    
    % % Check if model is open
    % if bdIsLoaded(modelName)
    %     close_system(modelName, 0);
    %     fprintf('Model "%s" was open and has been closed.\n', modelName);
    % else
    %     fprintf('Model "%s" is not open.\n', modelName);
    % end
    
    % Step 3 - Load dictionary
    step = step + 1;
    updateProgressBar(figHandle, step, totalSteps, 'Loading dictionary...');

    % Add ASW_Configuration folder to path if it exists
    % configFolder = fullfile(pwd, 'ASW_Configutation'); %ASW_Configuration
    % if isfolder(configFolder)
    %     addpath(genpath(configFolder));
    %     fprintf('Added "ASW_Configuration" folder to path.\n');
    % else
    %     fprintf('"ASW_Configuration" folder not found. Skipping path addition.\n');
    % end
    % chooseFolderASWConfigtoAddPath();
    
    %Handle Data Dictionary
    dataDictFile = [modelName '_DataDictionary.sldd'];
    if isfile(dataDictFile)
        % Close all open dictionaries first
        Simulink.data.dictionary.closeAll('-discard');
        fprintf('Closed all open data dictionaries.\n');
    else
        fprintf('Data dictionary file "%s" not found. Skipping dictionary closes.\n', dataDictFile);
    end
    
    % Construct the dictionary script name
    dictScript = [modelName '_DataDictionaryManagement.m'];
    % Construct the script name and run it
    if isfile(dictScript)
        fprintf('Executing dictionary script "%s" without running its first few lines (e.g., clear/close all).\n', dictScript);
    
        % Read script lines
        scriptLines = readlines(dictScript);
    
        % Skip the first N lines (adjust if needed)
        N = 4;
        scriptLines = scriptLines(N+1:end);
    
        % Join lines into a single script string
        scriptBody = join(scriptLines, newline);
    
        % Evaluate in caller workspace
        evalin('base', scriptBody);
    
        fprintf('Dictionary script executed with first %d lines skipped.\n', N);
    else
        warning('Expected data dictionary script not found: %s', dictScript);
        fprintf('Dictionary Management File "%s" not FOUND. Should terminate this.\n', dictScript);
        return;
    end
    
    % Step 4 - Generate code
    step = step + 1;
    updateProgressBar(figHandle, step, totalSteps, 'Generating AUTOSAR code...');

    %Generate code
    generateAutosarCodeOnly(modelName,matlabVer);

    % Step 5 - Export and zip
    step = step + 1;
    updateProgressBar(figHandle, step, totalSteps, 'Exporting and zipping...');

    if doZip
         exportCodeFiles(modelName);
         % exportConstParams(); %export const_params, optional
    else
        fprintf('Skipped code zip (checkbox not selected).\n');
    end

    % Final message
    updateProgressBar(figHandle, totalSteps, totalSteps, 'Task Completed!');
    msgbox('CodeGen success ! - quangln7 ','Success');

    % Close figure
    close(figHandle);

end

function generateAutosarCodeOnly(modelName,matlabVer)

    % Load the model if not already loaded
    if ~bdIsLoaded(modelName)
        load_system(modelName);
        fprintf('Model "%s" loaded.\n', modelName);
    end

    % Get active config set
    configSet = getActiveConfigSet(modelName);

    % Ensure System Target File is set to AUTOSAR
    stf = get_param(configSet, 'SystemTargetFile');
    if ~strcmp(stf, 'autosar.tlc')
        fprintf('Setting AUTOSAR system target file for model "%s"...\n', modelName);
        set_param(configSet, 'SystemTargetFile', 'autosar.tlc');
    end

    % Disable build/compilation (set 'GenerateCodeOnly' flag)
    set_param(modelName, 'GenCodeOnly', 'on');

    %Config param to make the model ready for ASAP2
    Simulink_model_cfg_fun();

    %Run the defineIntEnum if necessary to load some non-primitive
    %datatypes
    runSWCdefineIntEnumType(modelName);

    %Set param for typedef 2024a use config typedef
    if strcmp('(R2024a)',matlabVer)
        set_param(modelName, 'DataTypeReplacement', 'CoderTypedefs');  % only for 2024a
        set_param(modelName,'InstructionSetExtensions','None'); %No target Hardware specified here
        %---- this is optional, should add a tickbox to disable it----%
        set_param(modelName,'UnconnectedInputMsg','error');
        set_param(modelName,'UnconnectedOutputMsg','error');
        set_param(modelName,'UnconnectedLineMsg','error');
    end

    % Save model (optional)
    save_system(modelName);

    % Generate AUTOSAR code
    fprintf('Generating AUTOSAR C code for model "%s"...\n', modelName);
    rtwbuild(modelName);  % Will only generate code because 'GenCodeOnly' is ON
    
    % add gen a2l file if in 2024a --> should not manual gen a2l due to diffenrent format 
    % Generate A2L calibration files if MATLAB version is 2024a or newer
    % try
    %     v = ver('MATLAB');
    %     versionNum = sscanf(v.Version, '%d.%d');
    %     if versionNum(1) >= 25 %bypass 2021a
    %         fprintf('MATLAB 2024a or newer detected, generating A2L (calibration) files programmatically...\n');
    %         coder.asap2.export(modelName);
    %     else
    %         fprintf('MATLAB version < 2024a detected, no need to manually generate A2L.\n');
    %     end
    % catch ME
    %     fprintf('Failed to generate A2L files programmatically \n');
    %     errordlg(['Code Model generation failed: ' ME.message], 'Error');
    %     rethrow(ME);
    % end
    % 
    fprintf('AUTOSAR C code generation completed for "%s".\n', modelName);

end

function exportCodeFiles(modelName)

    % Define source and target folders
    srcFolder = [modelName '_autosar_rtw'];
    destFolder = [modelName '_code_delivery'];

    if ~isfolder(srcFolder)
        warning('Source folder "%s" does not exist. Skipping export.', srcFolder);
        return;
    end

    % Create destination folder if it doesn't exist
    if ~isfolder(destFolder)
        mkdir(destFolder);
    end

    % Define the list of files to copy
    fileList = {
        [modelName '.a2l'], ...
        [modelName '.c'], ...
        [modelName '_data.c'], ...
        [modelName '.h'], ...
        [modelName '_private.h'], ...
        [modelName '_types.h']
    };
    
    % Copy each file if it exists
    for i = 1:length(fileList)
        file = fileList{i};
        srcFile = fullfile(srcFolder, file);
        if isfile(srcFile)
            copyfile(srcFile, destFolder);
            fprintf('Copied: %s\n', file);
        else
            warning('File not found: %s', srcFile);
        end
    end

    %Also consider take const_params.c
    exportConstParams(destFolder);

    %Zip delivery folders
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');  % e.g., 20250825_111422
    zipName = sprintf('%s_code_delivery_%s.zip', modelName, timestamp);
    zip(zipName, destFolder);
    fprintf('Zipped delivery folder: %s\n', zipName);
    fprintf('Export complete. Files copied to: %s\n', destFolder);

end

function exportConstParams(destFolder)

    %define const param folder slprj new
    constFolder = fullfile(pwd, 'slprj\autosar\_sharedutils');
    
    if ~isfolder(constFolder)
        warning('Const folder "%s" does not exist. Skipping export.', constFolder);
        return;
    else
        constFile = fullfile(constFolder, 'const_params.c');
        if isfile(constFile)
            copyfile(constFile,destFolder);
            fprintf('Copied: const_params.c \n');
        else
            warning('File not found: const_params.c');
        end
    end

end
 
function updateProgressBar(fig, step, totalSteps, message)

    % Get or create progress bar axes
    ax = findobj(fig, 'Tag', 'ProgressBar');
    if isempty(ax)
        ax = axes('Parent', fig, 'Units','pixels', ...
                  'Position',[50 30 250 20], ...
                  'XTick',[], 'YTick',[], 'Box','on', ...
                  'Tag','ProgressBar');
        hold(ax,'on');
    end
    
    % Clear previous bar
    cla(ax);
    
    % Compute progress
    progress = step / totalSteps;
    
    % Draw filled green rectangle
    rectangle('Parent', ax, 'Position',[0 0 progress 1], ...
              'FaceColor',[0.2 0.8 0.2], 'EdgeColor','none');
    
    % Set limits
    xlim(ax,[0 1]); ylim(ax,[0 1]);
    
    % Add text message
    title(ax, sprintf('%s (%.0f%%)', message, progress*100), ...
          'FontSize',8, 'Color','k');
    
    drawnow;
    
end


function Simulink_model_cfg_fun()  % credit to JEE Xia Xia

    disableimplicitsignalresolution(bdroot);
    set_param(bdroot,'GenerateASAP2','on');
    set_param(bdroot,'UtilityFuncGeneration','Shared location');
    disp('Configuration modification completed')
    %set_param(bdroot,'Solver','ode3');
    
    %set_param(bdroot,'SystemTargetFile','ert.tlc');
    
    %set_param(bdroot,'GenerateMakefile','off');
    
    %set_param(bdroot,'GenerateComments','on');
    
    %set_param(bdroot,'GenerateTraceInfo','on');
    
    %set_param(bdroot,'GenerateReport','on');
    
    %set_param(bdroot,'LaunchReport','on');
    
    %set_param(bdroot,'GenerateSampleERTMain','off');
    
    %set_param(bdroot,'IncludeMdlTerminateFcn','off');
    
    %set_param(bdroot,'AlgebraicLoopMsg','error');

end

function matlabVer = checkMatlab()
    v = ver('MATLAB');
    % versionNum = sscanf(v.Release, '%s');
    % Ver =  versionNum(1); >= 25 %bypass 2021a
    matlabVer = v.Release;
end

function runGenASWCodePanelWithDir()
    
    % Add ASW_Configuration folder to path if it exists
    chooseFolderASWConfigtoAddPath();

    % Prompt user to select a folder
    selectedFolder = uigetdir(pwd, 'Please select Folder Containing SWC_*.slx Files');
    
    % Check if user canceled
    if isequal(selectedFolder, 0)
        disp('You cancelled folder selection.');
        return;
    end

    % Change to selected folder
    cd(selectedFolder);
    
    % Optionally, show current working directory
    fprintf('Running ASW Code Panel in: %s\n', selectedFolder);

    % Call the function to generate the panel
    genASWCodePanel_();

end

function chooseFolderASWConfigtoAddPath()

    %Prompt user to select ASW Configuration model to add to path
    aswConfigFolder = uigetdir(pwd,'Please select ASW Configuration folder to add to path');

    %Check if user cancelled the add path
    if isequal(aswConfigFolder,0)
        fprintf("You cancelled add to path ASW configuration folder ---> can not gen code, should terminate \n");
        return;
    end

    %add this selection to path (MUST ADD)
    if isfolder(aswConfigFolder)
        addpath(genpath(aswConfigFolder));
        fprintf('Added %s folder to path.\n',aswConfigFolder);
    else
        fprintf('"ASW_Configuration" folder not found. Skipping path addition.\n');
    end

end

function runSWCdefineIntEnumType(modelName)

    % Construct the filename based on modelName
    filename = [modelName, '_defineIntEnumTypes.m'];
    
    % Check if the file exists in the current directory
    if exist(filename, 'file') == 2
        % Run the file as a script
        run(filename);
        fprintf('File "%s" executed successfully.\n', filename);
    else
        fprintf('File "%s" does not exist in the current directory.\n', filename);
    end

end

