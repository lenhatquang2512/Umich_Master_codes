% /****************************************************************************
%  * Auto Generate shell model Simulink with Davinci Vector Arxml files
%  * Copyright (C) 2025
%  *
%  * @author Quang Nhat Le - v.quangln7@vinfast.vn
%  * @date   2025-23-8
%  ****************************************************************************/
function genASWModelPanel()

    clc; clear all mex; close all; format default;
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
    f = figure('Name','Select SWC Model', 'Position',[500 500 350 200], ...
               'MenuBar','none', 'NumberTitle','off', 'Resize','off');

    % Add popup menu
    uicontrol('Style','text', 'Position',[75 150 200 20], ...
              'String','Select a model to import:', 'FontWeight','bold');
    popup = uicontrol('Style','popupmenu', 'Position',[75 120 200 25], ...
                      'String', modelNames);

    % Add checkbox for code generation
    codeGenCheckbox = uicontrol('Style', 'checkbox', ...
                                'String', 'Generate AUTOSAR C Code after import', ...
                                'Position', [60 85 250 20], ...
                                'Value', 1);  % Checked by default


    % Add Generate button
    uicontrol('Style','pushbutton', 'String','Generate Model', ...
              'Position',[125 35 100 30], ...
              'Callback', @(src,event)generateCallback(popup, modelNames, f, codeGenCheckbox));

end

%This function is to handle code when 
function generateCallback(popup, modelNames, figHandle,codeGenCheckbox)
    % Get selected index and model name
    idx = get(popup, 'Value');
    modelName = modelNames{idx};
    
    %Remove slprj if it exists
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

    % Close figure
    close(figHandle);

    % Check if model is open
    if bdIsLoaded(modelName)
        close_system(modelName, 0); 
        fprintf('Model "%s" was open and has been closed.\n', modelName);
    else
        fprintf('Model "%s" is not open.\n', modelName);
    end

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
        componentPath = ['/ComponentTypes/' modelName];
        obj = arxml.importer(arxmlFiles);
        % names = getComponentNames(obj);
        createComponentAsModel(obj, componentPath, ...
            'ModelPeriodicRunnablesAs', 'FunctionCallSubsystem');

        % Run model config
        Simulink_model_cfg;
        fprintf('Model "%s" successfully generated.\n', modelName);

        %Generate code 
        if doCodeGen
            generateAutosarCodeOnly(modelName);
        else
            fprintf('Skipped AUTOSAR code generation (checkbox not selected).\n');
        end


    catch ME
        errordlg(['Code Model generation failed: ' ME.message], 'Error');
        rethrow(ME);
    end
end

function generateAutosarCodeOnly(modelName)
    % Example: generateAutosarCodeOnly('SWC_MDF');

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

    % Save model (optional)
    save_system(modelName);

    % Generate AUTOSAR code
    fprintf('Generating AUTOSAR C code for model "%s"...\n', modelName);
    rtwbuild(modelName);  % Will only generate code because 'GenCodeOnly' is ON

    fprintf('AUTOSAR C code generation completed for "%s".\n', modelName);
end