% /****************************************************************************
%  * Auto Generate shell model and C code from Simulink with Davinci Vector Arxml files
%  * Copyright (C) 2025
%  *
%  * @author Quang Nhat Le - v.quangln7@vinfast.vn
%  * @date   2025-23-8
%  ****************************************************************************/
clc; clear all; bdclose all; format default; restoredefaultpath;
% global matlabVer;
% genASWModelPanel_();
runGenASWModelPanelWithDir();

function genASWModelPanel_()

    % clc; clear all mex; close all; format default; restoredefaultpath;
    % Available model names
    % modelNames = {'SWC_VCF', 'SWC_MDF'};

    % Auto-detect available SWC_*.arxml files in the current folder
    swcFiles = dir('SWC_*.arxml');
    if isempty(swcFiles)
        errordlg('No SWC_*.arxml files found in the current folder.', 'Error');
        return;
    end
    modelNames = cellfun(@(f) erase(f, '.arxml'), {swcFiles.name}, 'UniformOutput', false);

    % Create figure
    f = figure('Name','Vinfast ASW Autosar Gen', 'Position',[500 400 350 300], ...
               'MenuBar','none', 'NumberTitle','off', 'Resize','off');

    % Add popup menu
    uicontrol('Style','text', 'Position',[75 250 200 20], ...
              'String','Select a model to import ', 'FontWeight','bold');
    popup = uicontrol('Style','popupmenu', 'Position',[75 200 200 25], ...
                      'String', modelNames);

    % Add checkbox for code generation
    codeGenCheckbox = uicontrol('Style', 'checkbox', ...
                                'String', 'Generate AUTOSAR C Code after import', ...
                                'Position', [75 170 250 20], ...
                                'Value', 1);  % Checked by default
    zipCheckbox = uicontrol('Style', 'checkbox', ...
                                'String', 'Zip code delivery after generating', ...
                                'Position', [75 140 250 20], ...
                                'Value', 1);  % Checked by default


    % Add Generate button
    uicontrol('Style','pushbutton', 'String','Generate Shell / Update Model', ...
              'Position',[55 80 250 30], ...
              'Callback', @(src,event)generateCallback(popup, modelNames, f, codeGenCheckbox, zipCheckbox));

    % === Add Progress Bar (initially empty) ===
    ax = axes('Parent', f, 'Units','pixels', ...
              'Position',[25 30 300 20], ...
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

%This function is to handle code when 
function generateCallback(popup, modelNames, figHandle,codeGenCheckbox, zipCheckbox)
    % Get selected index and model name
    idx = get(popup, 'Value');
    modelName = modelNames{idx};
    
    % define total steps of workflow
    totalSteps = 5;   
    step = 1;

    %Create progress bar initially
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
    
    %Check if user selected code generation
    doCodeGen = get(codeGenCheckbox, 'Value');
    doZip = get(zipCheckbox, 'Value');

    %Check matlab ver
    matlabVer = checkMatlab();

    % Check if model is open
    %     if bdIsLoaded(modelName)
    %         close_system(modelName, 0); 
    %         fprintf('Model "%s" was open and has been closed.\n', modelName);
    %     else
    %         fprintf('Model "%s" is not open.\n', modelName);
    %     end

    % ARXML files
    arxmlFiles = {
        [modelName '.arxml'], ...
        'DataTypes.arxml', ...
        'PortInterfaces.arxml', ...
        'FiM_swc.arxml', ...
        'Dem_swc.arxml', ...
        'IoHwAb_swc.arxml', ...
        'PlatformTypes_AR4.arxml', ...
        'ComM_swc.arxml', ...
        'BswM_swc.arxml'
        % Add more ARXML files if needed, currently 9 files
    };

    % Check for missing files
    missingFiles = arxmlFiles(~cellfun(@isfile, arxmlFiles));
    if ~isempty(missingFiles)
        msg = sprintf('Missing ARXML files:\n%s', strjoin(missingFiles, '\n'));
        errordlg(msg, 'Missing Files');
        return;
    end
   
    % Import ARXML and generate model
    try

        % Import ARXML and generate model
        step = step + 1;
        updateProgressBar(figHandle, step, totalSteps, 'Import ARXML and generate model...');
        %check whether model is existed or not
        if isfile([modelName '.slx'])  % if existed then update the model
            
            %load the data dictionary if the model is already existed 
            loadDataDictionary(modelName, 4); % skip first 4 lines 
            
            %Start open and update the model
            open_system(modelName);
            ar2 = arxml.importer(arxmlFiles);
            updateModel(ar2,modelName);  
            Simulink_model_cfg_fun();

            fprintf('Model "%s" successfully getting updated after changing ARXML files.\n', modelName);

        else % just gen the shell model
            componentPath = ['/ComponentTypes/' modelName];
            obj = arxml.importer(arxmlFiles);
            % names = getComponentNames(obj);
            createComponentAsModel(obj, componentPath, ...
                'ModelPeriodicRunnablesAs', 'FunctionCallSubsystem');
            % Run model config
            % Simulink_model_cfg;
            Simulink_model_cfg_fun();

            fprintf('Model "%s" successfully generated.\n', modelName);
        end

        %Generate code 
        step = step + 1;
        updateProgressBar(figHandle, step, totalSteps, 'Generating AUTOSAR code...');
        if doCodeGen
            generateAutosarCodeOnly(modelName,matlabVer);
        else
            fprintf('Skipped AUTOSAR code generation (checkbox not selected).\n');
        end
        
       % Export and zip
       step = step + 1;
       updateProgressBar(figHandle, step, totalSteps, 'Exporting and zipping...');
       if doZip
            exportCodeFiles(modelName);
       else
            fprintf('Skipped code zip (checkbox not selected).\n');
       end

       % Final message
       updateProgressBar(figHandle, totalSteps, totalSteps, 'Task Completed!');
       msgbox('CodeGen success ! - quangln7 ','Success');

        % Close figure
        close(figHandle);

    catch ME
        close(figHandle);  %clode GUI when fail
        errordlg(['Code Model generation failed: ' ME.message], 'Error');
        rethrow(ME);
    end
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

    %Set param for typedef 2024a use config typedef
    if strcmp('(R2024a)',matlabVer)
        set_param(modelName, 'DataTypeReplacement', 'CoderTypedefs');  % only for 2024a
        set_param(modelName,'InstructionSetExtensions','None'); %No target Hardware specified here
        %---- this is optional, should add a tickbox to disable it----%
        set_param(modelName,'UnconnectedInputMsg','error');
        set_param(modelName,'UnconnectedOutputMsg','error');
        set_param(modelName,'UnconnectedLineMsg','error');
    end

    %Optional update Autosarconfig 
    updateAutosarModelConfig(modelName);

    % Save model (optional)
    save_system(modelName);

    % Generate AUTOSAR code
    fprintf('Generating AUTOSAR C code for model "%s"...\n', modelName);
    rtwbuild(modelName);  % Will only generate code because 'GenCodeOnly' is ON

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


%todo
function updateAutosarModelConfig(modelName)

    % modelName = 'SWC_TPC'; % Or any component name
    % set_param(modelName, 'AUTOSARComponentName', ['AUTOSAR.' modelName '.' modelName]);
    arProps = autosar.api.getAUTOSARProperties(modelName);
    % set(arProps,'XmlOptions','ArxmlFilePackaging','SingleFile');
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

function runGenASWModelPanelWithDir()

    % Prompt user to select a folder
    selectedFolder = uigetdir(pwd, 'Select Folder Containing SWC_*.arxml Files');
    
    % Check if user canceled
    if isequal(selectedFolder, 0)
        disp('User canceled folder selection.');
        return;
    end

    % Change to selected folder
    cd(selectedFolder);
    
    % Optionally, show current working directory
    fprintf('Running ASW Model Panel in: %s\n', selectedFolder);

    % Call the function to generate the panel
    genASWModelPanel_();

end


function loadDataDictionary(modelName, lineskip)

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
        N = lineskip;
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

end

%Todo a function to load all necessary arxml files 