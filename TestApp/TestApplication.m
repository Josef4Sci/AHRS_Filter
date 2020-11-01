classdef TestApplication < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure            matlab.ui.Figure
        Button              matlab.ui.control.Button
        UIAxes              matlab.ui.control.UIAxes
        LabelDropDown       matlab.ui.control.Label
        DropDown            matlab.ui.control.DropDown
        Label               matlab.ui.control.Label
        DropDown2           matlab.ui.control.DropDown
        Button2             matlab.ui.control.Button
        Button3             matlab.ui.control.Button
        Panel               matlab.ui.container.Panel
        NumericEditField    matlab.ui.control.NumericEditField
        NumericEditField2   matlab.ui.control.NumericEditField
        NumericEditField3   matlab.ui.control.NumericEditField
        Button4             matlab.ui.control.Button
        Panel2              matlab.ui.container.Panel
        NumericEditField4   matlab.ui.control.NumericEditField
        NumericEditField5   matlab.ui.control.NumericEditField
        NumericEditField6   matlab.ui.control.NumericEditField
        Panel3              matlab.ui.container.Panel
        ChosedatasetLabel   matlab.ui.control.Label
        DropDown3           matlab.ui.control.DropDown
        CheckBox            matlab.ui.control.CheckBox
        CheckBox2           matlab.ui.control.CheckBox
        CheckBox7           matlab.ui.control.CheckBox
        CheckBox8           matlab.ui.control.CheckBox
        TextArea            matlab.ui.control.TextArea
        UIAxes2             matlab.ui.control.UIAxes
        UIAxes3             matlab.ui.control.UIAxes
        Button5             matlab.ui.control.Button
        Button6             matlab.ui.control.Button
        Panel4              matlab.ui.container.Panel
        NumericEditField7   matlab.ui.control.NumericEditField
        NumericEditField8   matlab.ui.control.NumericEditField
        NumericEditField9   matlab.ui.control.NumericEditField
        Panel5              matlab.ui.container.Panel
        NumericEditField10  matlab.ui.control.NumericEditField
        NumericEditField11  matlab.ui.control.NumericEditField
        NumericEditField12  matlab.ui.control.NumericEditField
        Label2              matlab.ui.control.Label
        DropDown4           matlab.ui.control.DropDown
        Label3              matlab.ui.control.Label
        DropDown5           matlab.ui.control.DropDown
        UIAxes4             matlab.ui.control.UIAxes
        UIAxes5             matlab.ui.control.UIAxes
        CheckBox3           matlab.ui.control.CheckBox
        CheckBox4           matlab.ui.control.CheckBox
        CheckBox5           matlab.ui.control.CheckBox
        CheckBox6           matlab.ui.control.CheckBox
        IncludeintestLabel  matlab.ui.control.Label
    end

    
    properties (Access = public)
        AHRS1; % Description
        AHRS2;        
        AHRS3;
        AHRS4;
        
        AHRS;
        
        errorProgress1;
        intProg1=1;
        errorProgress2;
        intProg2=1;
        
        errorProgress3;
        intProg3=1;
        errorProgress4;
        intProg4=1;
        
        stopOpt=0;
        
        dataset=1;
        RMS=0;
        IMU=0;
        exceptPeak=0;
        
        u1;
        u2;
        u3;
        u4;
        
        M1;
        M2;
        M3;
        M4;
        
        optimAttempts=0;
        optimFreezeParam=0;
    end
    
    methods (Access = private)
        function valueMathodChange(app,className, num, val1, val2, val3)
            switch(className)
                case 'MadgwickAHRS3'
                    app.AHRS(num).Beta=val1;
                case 'MadgwickAHRSclanek'
                    app.AHRS1.Beta=val1;
                case 'JustaAHRSPureFast'
                    app.AHRS(num).gain=val1;
                    app.AHRS(num).wAcc=val2;
                    app.AHRS(num).wMag=val3;
                case 'Valenti_AHRS'
                    app.AHRS(num).wAcc=val1;
                    app.AHRS(num).wMag=val2;
                case 'JustaAHRSPureFastConstantCorr'
                    app.AHRS(num).wAcc=val1;
                    app.AHRS(num).wMag=val2;
                case 'JustaAHRSPureFastLinearCorr'
                    app.AHRS(num).gain=val1;
                    app.AHRS(num).wAcc=val2;
                    app.AHRS(num).wMag=val3;
                case 'JustaAHRSPure'
                    app.AHRS(num).wAcc=val1;
                    app.AHRS(num).wMag=val2;
                case 'JinWuKF_AHRSreal2'
                    app.AHRS(num).Sigma_g=diag([1 1 1])*val1;
                    app.AHRS(num).Sigma_a=diag([1 1 1])*val2;
                    app.AHRS(num).Sigma_m=diag([1 1 1])*val3;
                case 'YoungSooSuh_AHRS'
                    app.AHRS(num).rg=val1;
                    app.AHRS(num).ra=val2;
            end
        end
        
        
        function valChange1(app,src,event)            
            className=class(app.AHRS1);
            v1=app.NumericEditField.Value;
            v2=app.NumericEditField2.Value;
            v3=app.NumericEditField3.Value;
            valueMathodChange(app, className, 1, v1, v2, v3);
        end
        
        function valChange2(app,src,event)
            className=class(app.AHRS2);
            v1=app.NumericEditField4.Value;
            v2=app.NumericEditField5.Value;
            v3=app.NumericEditField6.Value;
            valueMathodChange(app, className, 2, v1, v2, v3);
        end
        
        function valChange3(app,src,event)
            className=class(app.AHRS3);
            v1=app.NumericEditField7.Value;
            v2=app.NumericEditField8.Value;
            v3=app.NumericEditField9.Value;
            valueMathodChange(app, className, 3, v1, v2, v3);            
        end
        
        function valChange4(app,src,event)
            className=class(app.AHRS4);            
            v1=app.NumericEditField10.Value;
            v2=app.NumericEditField11.Value;
            v3=app.NumericEditField12.Value;
            valueMathodChange(app, className, 4, v1, v2, v3);            
        end
        
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.AHRS(1) = MadgwickAHRS3();
            app.AHRS(2) = JustaAHRSPureFast();
            app.AHRS(3) = Valenti_AHRS();
            app.AHRS(4) = JinWuKF_AHRSreal2();
            
            
            app.NumericEditField.Value=app.AHRS(1).Beta;
            app.NumericEditField4.Value=app.AHRS(2).gain;
            app.NumericEditField5.Value=app.AHRS(2).wAcc;
            app.NumericEditField6.Value=app.AHRS(2).wMag;
            app.NumericEditField7.Value=app.AHRS(3).wAcc;
            app.NumericEditField8.Value=app.AHRS(3).wMag;
            app.NumericEditField10.Value=app.AHRS(4).Sigma_a(1,1);
            app.NumericEditField11.Value=app.AHRS(4).Sigma_m(1,1);
            
            app.NumericEditField.ValueChangedFcn=@app.valChange1;
            app.NumericEditField2.ValueChangedFcn=@app.valChange1;
            app.NumericEditField3.ValueChangedFcn=@app.valChange1;
            app.NumericEditField4.ValueChangedFcn=@app.valChange2;
            app.NumericEditField5.ValueChangedFcn=@app.valChange2;
            app.NumericEditField6.ValueChangedFcn=@app.valChange2;            
            app.NumericEditField7.ValueChangedFcn=@app.valChange3;
            app.NumericEditField8.ValueChangedFcn=@app.valChange3;
            app.NumericEditField9.ValueChangedFcn=@app.valChange3;
            app.NumericEditField10.ValueChangedFcn=@app.valChange4;
            app.NumericEditField11.ValueChangedFcn=@app.valChange4;
            app.NumericEditField12.ValueChangedFcn=@app.valChange4;
            
            
        end

        % Button pushed function: Button
        function startApp(app, event)

            evalin('base','clear');
            enabled=[0 0 0 0];
            if(app.CheckBox3.Value)
                enabled(1)=1;
            end
            if(app.CheckBox4.Value)
                enabled(2)=1;
            end
            if(app.CheckBox5.Value)
                enabled(3)=1;
            end
            if(app.CheckBox6.Value)
                enabled(4)=1;
            end
            
            res = {''};
            
            
            for i=1:4
                if(enabled(i))
                    switch(i)
                        case 1
                            [app.M1,app.u1,time]=datasetTestingFunctionSingle(app.AHRS1,app.dataset,app.RMS,app.IMU, app.exceptPeak);
                            res{end+1} =strcat('Method 1 error: ',num2str(app.M1));
                            assignin('base','M1uhel',app.u1');
                        case 2
                            [app.M2,app.u2,time]=datasetTestingFunctionSingle(app.AHRS2,app.dataset,app.RMS,app.IMU, app.exceptPeak);
                            res{end+1} =strcat('Method 2 error: ',num2str(app.M2));
                            assignin('base','M2uhel',app.u2');
                        case 3
                            [app.M3,app.u3,time]=datasetTestingFunctionSingle(app.AHRS3,app.dataset,app.RMS,app.IMU, app.exceptPeak);
                            res{end+1} =strcat('Method 3 error: ',num2str(app.M3));
                            assignin('base','M3uhel',app.u3');
                        case 4
                            [app.M4,app.u4,time]=datasetTestingFunctionSingle(app.AHRS4,app.dataset,app.RMS,app.IMU, app.exceptPeak);
                            res{end+1} =strcat('Method 4 error: ',num2str(app.M4));
                            assignin('base','M4uhel',app.u4');
                    end
                end
            end
            assignin('base','time',time);
            %             PlotErrors;
            app.TextArea.Value=res;
        end

        % Value changed function: DropDown
        function methodOneChanged(app, event)
            value = app.DropDown.Value;
            app.errorProgress1=0;
            app.intProg1=1;
            
            switch(value)
                case 'MadgwickModif'
                    app.AHRS1 = MadgwickAHRS3();
                case 'Madgwick'
                    app.AHRS1 = MadgwickAHRSclanek();
                case 'JustaFast'
                    app.AHRS1 = JustaAHRSPureFast();
                case 'JustaPure'
                    app.AHRS1 = JustaAHRSPure();
                case 'JustaLinear'
                    app.AHRS1 = JustaAHRSPureFastLinearCorr();
                case 'JustaConstant'
                    app.AHRS1 = JustaAHRSPureFastConstantCorr();
                case 'Valenti'
                    app.AHRS1 = Valenti_AHRS();
                case 'Guo'
                    app.AHRS1 = JinWuKF_AHRSreal2();
                case 'YoungSooSuh'
                    app.AHRS1 = YoungSooSuh_AHRS();
            end
        end

        % Value changed function: DropDown2
        function methodTwoChanged(app, event)
            value = app.DropDown2.Value;
            
            app.errorProgress2=0;
            app.intProg2=1;
            switch(value)
                case 'MadgwickModif'
                    app.AHRS2 = MadgwickAHRS3();
                case 'Madgwick'
                    app.AHRS2 = MadgwickAHRSclanek();
                case 'JustaLinear'
                    app.AHRS2 = JustaAHRSPureFastLinearCorr();
                case 'JustaConstant'
                    app.AHRS2 = JustaAHRSPureFastConstantCorr();
                case 'JustaFast'
                    app.AHRS2 = JustaAHRSPureFast();
                case 'JustaPure'
                    app.AHRS2 = JustaAHRSPure();
                case 'Valenti'
                    app.AHRS2 = Valenti_AHRS();
                case 'Guo'
                    app.AHRS2 = JinWuKF_AHRSreal2();
                case 'YoungSooSuh'
                    app.AHRS2 = YoungSooSuh_AHRS();
            end
        end

        % Button pushed function: Button2
        function OptimizeM1(app, event)
            
            app.Button2.Text='Wait';
            app.Button4.Text= 'Stop Optimization';
            while(true)
                [app.AHRS1,e,param,app.optimAttempts,app.optimFreezeParam]=optimFilterParams(app.AHRS1,app.RMS,app.dataset,app.optimAttempts,app.optimFreezeParam,app.IMU);
                app.errorProgress1(app.intProg1)=e;
                app.intProg1=app.intProg1+1;
                
                plot(app.UIAxes,1:length(app.errorProgress1),app.errorProgress1);
                hold(app.UIAxes,'on');
                plot(app.UIAxes,1:length(app.errorProgress2),app.errorProgress2);
                plot(app.UIAxes,1:length(app.errorProgress3),app.errorProgress3);
                plot(app.UIAxes,1:length(app.errorProgress4),app.errorProgress4);
                hold(app.UIAxes,'off');
                plot(app.UIAxes2,1:length(app.errorProgress1),app.errorProgress1);
                
                app.NumericEditField.Value=param(1);
                app.NumericEditField2.Value=param(2);
                app.NumericEditField3.Value=param(3);
                pause(0.5);
                if(app.stopOpt)
                    app.Button4.Text = '';
                    app.stopOpt=0;
                    break;
                end
            end
            app.optimAttempts=0;
            app.optimFreezeParam=0;
            
            app.Button2.Text='Optimize';
            %             ylabel(app.UIAxes,'muj modrej')
            %             hold(app.UIAxes);
            %             plot(app.UIAxes,movmean(uhelMadg(1:(myEnd-start)),300));
            %             holdoff(app.UIAxes);
        end

        % Button pushed function: Button4
        function stop(app, event)
            app.stopOpt=1;
        end

        % Button pushed function: Button3
        function OptimizeM2(app, event)
            app.Button3.Text='Wait';
            app.Button4.Text= 'Stop Optimization';
            while(true)
                [app.AHRS2,e,param,app.optimAttempts,app.optimFreezeParam]=optimFilterParams(app.AHRS2,app.RMS,app.dataset,app.optimAttempts,app.optimFreezeParam,app.IMU);
                
                app.errorProgress2(app.intProg2)=e;
                app.intProg2=app.intProg2+1;
                plot(app.UIAxes,1:length(app.errorProgress2),app.errorProgress2);
                hold(app.UIAxes,'on');
                plot(app.UIAxes,1:length(app.errorProgress1),app.errorProgress1);
                plot(app.UIAxes,1:length(app.errorProgress3),app.errorProgress3);
                plot(app.UIAxes,1:length(app.errorProgress4),app.errorProgress4);
                hold(app.UIAxes,'off');
                plot(app.UIAxes3,1:length(app.errorProgress2),app.errorProgress2);
                
                app.NumericEditField4.Value=param(1);
                app.NumericEditField5.Value=param(2);
                app.NumericEditField6.Value=param(3);
                pause(0.5);
                if(app.stopOpt)
                    app.Button4.Text = '';
                    app.stopOpt=0;
                    break;
                end
            end
            app.optimAttempts=0;
            app.optimFreezeParam=0;
            
            app.Button3.Text='Optimize';
        end

        % Value changed function: DropDown3
        function datasetChanged(app, event)
            value = app.DropDown3.Value;
            switch(value)
                case 'Set1'
                    app.dataset=1;
                case 'Set2'
                    app.dataset=2;
                case 'Set12'
                    app.dataset=3;
                case 'Set123'
                    app.dataset=4;
                case 'ALSdataset'
                    app.dataset=5;
                case 'Synthetic2'
                    app.dataset=6;
                case 'Synthetic3'
                    app.dataset=7;
            end
            app.errorProgress1=0;
            app.intProg1=1;
            app.errorProgress2=0;
            app.intProg2=1;
            app.errorProgress3=0;
            app.intProg3=1;
            app.errorProgress4=0;
            app.intProg4=1;
        end

        % Value changed function: CheckBox
        function RMSchanged(app, event)
            app.RMS = app.CheckBox.Value;
            app.errorProgress1=0;
            app.intProg1=1;
            app.errorProgress2=0;
            app.intProg2=1;
            
            app.errorProgress3=0;
            app.intProg3=1;
            app.errorProgress4=0;
            app.intProg4=1;
        end

        % Value changed function: NumericEditField4
        function valChanged2(app, event)
            
        end

        % Value changed function: NumericEditField
        function valChanged1(app, event)
            
        end

        % Value changed function: DropDown4
        function Method3Changed(app, event)
            value = app.DropDown4.Value;
            
            app.errorProgress3=0;
            app.intProg3=1;
            switch(value)
                case 'MadgwickModif'
                    app.AHRS3 = MadgwickAHRS3();
                case 'Madgwick'
                    app.AHRS3 = MadgwickAHRSclanek();
                case 'JustaFast'
                    app.AHRS3 = JustaAHRSPureFast();
                case 'JustaPure'
                    app.AHRS3 = JustaAHRSPure();
                
                case 'JustaLinear'
                    app.AHRS3 = JustaAHRSPureFastLinearCorr();
                case 'JustaConstant'
                    app.AHRS3 = JustaAHRSPureFastConstantCorr();
                case 'Valenti'
                    app.AHRS3 = Valenti_AHRS();
                case 'Guo'
                    app.AHRS3 = JinWuKF_AHRSreal2();
                case 'YoungSooSuh'
                    app.AHRS3 = YoungSooSuh_AHRS();
                case 'Wilson'
                    app.AHRS3 = Wilson_Madgwick_AHRS();
                    
            end
        end

        % Value changed function: DropDown5
        function Method4Changed(app, event)
            value = app.DropDown5.Value;
            app.errorProgress4=0;
            app.intProg4=1;
            switch(value)
                case 'MadgwickModif'
                    app.AHRS4 = MadgwickAHRS3();
                case 'Madgwick'
                    app.AHRS4 = MadgwickAHRSclanek();
                case 'JustaFast'
                    app.AHRS4 = JustaAHRSPureFast();
                case 'JustaPure'
                    app.AHRS4 = JustaAHRSPure();
                case 'JustaLinear'
                    app.AHRS4 = JustaAHRSPureFastLinearCorr();
                case 'JustaConstant'
                    app.AHRS4 = JustaAHRSPureFastConstantCorr();
                case 'Valenti'
                    app.AHRS4 = Valenti_AHRS();
                case 'Guo'
                    app.AHRS4 = JinWuKF_AHRSreal2();
                case 'YoungSooSuh'
                    app.AHRS4 = YoungSooSuh_AHRS();
                case 'Admirall'
                    app.AHRS4 = AdmirallWilsonAHRS();
            end
        end

        % Button pushed function: Button5
        function OptimizeM3(app, event)
            app.Button5.Text='Wait';
            app.Button4.Text= 'Stop Optimization';
            while(true)
                [app.AHRS3,e,param,app.optimAttempts,app.optimFreezeParam]=optimFilterParams(app.AHRS3,app.RMS,app.dataset,app.optimAttempts,app.optimFreezeParam,app.IMU);
                
                app.errorProgress3(app.intProg3)=e;
                app.intProg3=app.intProg3+1;
                plot(app.UIAxes,1:length(app.errorProgress3),app.errorProgress3);
                hold(app.UIAxes,'on');
                plot(app.UIAxes,1:length(app.errorProgress1),app.errorProgress1);
                plot(app.UIAxes,1:length(app.errorProgress2),app.errorProgress2);
                plot(app.UIAxes,1:length(app.errorProgress4),app.errorProgress4);
                hold(app.UIAxes,'off');
                plot(app.UIAxes4,1:length(app.errorProgress3),app.errorProgress3);
                
                app.NumericEditField7.Value=param(1);
                app.NumericEditField8.Value=param(2);
                app.NumericEditField9.Value=param(3);
                pause(0.5);
                if(app.stopOpt)
                    app.Button4.Text = '';
                    app.stopOpt=0;
                    break;
                end
            end
            app.optimAttempts=0;
            app.optimFreezeParam=0;
            
            app.Button5.Text='Optimize';
        end

        % Button pushed function: Button6
        function OptimizeM4(app, event)
            app.Button6.Text='Wait';
            app.Button4.Text= 'Stop Optimization';
            while(true)
                [app.AHRS4,e,param,app.optimAttempts,app.optimFreezeParam]=optimFilterParams(app.AHRS4,app.RMS,app.dataset,app.optimAttempts,app.optimFreezeParam,app.IMU);
                
                app.errorProgress4(app.intProg4)=e;
                app.intProg4=app.intProg4+1;
                plot(app.UIAxes,1:length(app.errorProgress4),app.errorProgress4);
                hold(app.UIAxes,'on');
                plot(app.UIAxes,1:length(app.errorProgress1),app.errorProgress1);
                plot(app.UIAxes,1:length(app.errorProgress2),app.errorProgress2);
                plot(app.UIAxes,1:length(app.errorProgress3),app.errorProgress3);
                hold(app.UIAxes,'off');
                plot(app.UIAxes5,1:length(app.errorProgress4),app.errorProgress4);
                
                app.NumericEditField10.Value=param(1);
                app.NumericEditField11.Value=param(2);
                app.NumericEditField12.Value=param(3);
                pause(0.5);
                if(app.stopOpt)
                    app.Button4.Text = '';
                    app.stopOpt=0;
                    break;
                end
            end
            app.optimAttempts=0;
            app.optimFreezeParam=0;
            
            app.Button6.Text='Optimize';
        end

        % Value changed function: CheckBox7
        function IMUchange(app, event)
            value = app.CheckBox7.Value;
            app.IMU=value;
        end

        % Value changed function: CheckBox8
        function ExceptChange(app, event)
            app.exceptPeak=app.CheckBox8.Value;
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.AutoResizeChildren = 'off';
            app.UIFigure.Position = [51 61 1009 889];
            app.UIFigure.Name = 'Compare AHRS method GUI';

            % Create Button
            app.Button = uibutton(app.UIFigure, 'push');
            app.Button.ButtonPushedFcn = createCallbackFcn(app, @startApp, true);
            app.Button.Position = [238 67 100 22];
            app.Button.Text = 'Start test';

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            title(app.UIAxes, 'Campare optimization')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            app.UIAxes.GridAlpha = 0.15;
            app.UIAxes.MinorGridAlpha = 0.25;
            app.UIAxes.Box = 'on';
            app.UIAxes.Position = [149 173 475 324];

            % Create LabelDropDown
            app.LabelDropDown = uilabel(app.UIFigure);
            app.LabelDropDown.HorizontalAlignment = 'right';
            app.LabelDropDown.VerticalAlignment = 'top';
            app.LabelDropDown.Position = [87 707 48 15];
            app.LabelDropDown.Text = 'Method1';

            % Create DropDown
            app.DropDown = uidropdown(app.UIFigure);
            app.DropDown.Items = {'Madgwick', 'JustaFast', 'Valenti', 'Guo', 'MadgwickModif', 'JustaPure', 'JustaLinear', 'JustaConstant', 'YoungSooSuh'};
            app.DropDown.ValueChangedFcn = createCallbackFcn(app, @methodOneChanged, true);
            app.DropDown.Position = [142 703 108 22];
            app.DropDown.Value = 'MadgwickModif';

            % Create Label
            app.Label = uilabel(app.UIFigure);
            app.Label.HorizontalAlignment = 'right';
            app.Label.VerticalAlignment = 'top';
            app.Label.Position = [87 644 48 15];
            app.Label.Text = 'Method2';

            % Create DropDown2
            app.DropDown2 = uidropdown(app.UIFigure);
            app.DropDown2.Items = {'Madgwick', 'JustaFast', 'Valenti', 'Guo', 'MadgwickModif', 'JustaPure', 'JustaLinear', 'JustaConstant', 'YoungSooSuh'};
            app.DropDown2.ValueChangedFcn = createCallbackFcn(app, @methodTwoChanged, true);
            app.DropDown2.Position = [142 642 108 20];
            app.DropDown2.Value = 'JustaFast';

            % Create Button2
            app.Button2 = uibutton(app.UIFigure, 'push');
            app.Button2.ButtonPushedFcn = createCallbackFcn(app, @OptimizeM1, true);
            app.Button2.Position = [258 703 100 22];
            app.Button2.Text = 'Optimize';

            % Create Button3
            app.Button3 = uibutton(app.UIFigure, 'push');
            app.Button3.ButtonPushedFcn = createCallbackFcn(app, @OptimizeM2, true);
            app.Button3.Position = [258 641 100 22];
            app.Button3.Text = 'Optimize';

            % Create Panel
            app.Panel = uipanel(app.UIFigure);
            app.Panel.AutoResizeChildren = 'off';
            app.Panel.Title = 'Panel';
            app.Panel.Position = [392 701 260 50];

            % Create NumericEditField
            app.NumericEditField = uieditfield(app.Panel, 'numeric');
            app.NumericEditField.ValueChangedFcn = createCallbackFcn(app, @valChanged1, true);
            app.NumericEditField.Position = [9 4 61 22];

            % Create NumericEditField2
            app.NumericEditField2 = uieditfield(app.Panel, 'numeric');
            app.NumericEditField2.Position = [95 4 70 22];

            % Create NumericEditField3
            app.NumericEditField3 = uieditfield(app.Panel, 'numeric');
            app.NumericEditField3.Position = [190 4 64 22];

            % Create Button4
            app.Button4 = uibutton(app.UIFigure, 'push');
            app.Button4.ButtonPushedFcn = createCallbackFcn(app, @stop, true);
            app.Button4.Position = [257 750 102 22];
            app.Button4.Text = '';

            % Create Panel2
            app.Panel2 = uipanel(app.UIFigure);
            app.Panel2.AutoResizeChildren = 'off';
            app.Panel2.Title = 'Panel';
            app.Panel2.Position = [392 635 260 50];

            % Create NumericEditField4
            app.NumericEditField4 = uieditfield(app.Panel2, 'numeric');
            app.NumericEditField4.ValueChangedFcn = createCallbackFcn(app, @valChanged2, true);
            app.NumericEditField4.Position = [9 4 61 22];

            % Create NumericEditField5
            app.NumericEditField5 = uieditfield(app.Panel2, 'numeric');
            app.NumericEditField5.Position = [95 4 70 22];

            % Create NumericEditField6
            app.NumericEditField6 = uieditfield(app.Panel2, 'numeric');
            app.NumericEditField6.Position = [190 4 64 22];

            % Create Panel3
            app.Panel3 = uipanel(app.UIFigure);
            app.Panel3.AutoResizeChildren = 'off';
            app.Panel3.Title = 'Panel';
            app.Panel3.Position = [218 793 579 72];

            % Create ChosedatasetLabel
            app.ChosedatasetLabel = uilabel(app.Panel3);
            app.ChosedatasetLabel.HorizontalAlignment = 'right';
            app.ChosedatasetLabel.VerticalAlignment = 'top';
            app.ChosedatasetLabel.Position = [6 13 83 22];
            app.ChosedatasetLabel.Text = 'Chose dataset';

            % Create DropDown3
            app.DropDown3 = uidropdown(app.Panel3);
            app.DropDown3.Items = {'Set1', 'Set2', 'Set12', 'Set123', 'ALSdataset', 'Synthetic2', 'Synthetic3'};
            app.DropDown3.ValueChangedFcn = createCallbackFcn(app, @datasetChanged, true);
            app.DropDown3.Position = [104 16 100 22];
            app.DropDown3.Value = 'Set1';

            % Create CheckBox
            app.CheckBox = uicheckbox(app.Panel3);
            app.CheckBox.ValueChangedFcn = createCallbackFcn(app, @RMSchanged, true);
            app.CheckBox.Tooltip = {'Mean square error'};
            app.CheckBox.Text = 'MSE';
            app.CheckBox.Position = [302 19 45 16];

            % Create CheckBox2
            app.CheckBox2 = uicheckbox(app.Panel3);
            app.CheckBox2.Tooltip = {'The error estimated from euler angles.'};
            app.CheckBox2.Text = 'Euler';
            app.CheckBox2.Position = [359 19 48 16];

            % Create CheckBox7
            app.CheckBox7 = uicheckbox(app.Panel3);
            app.CheckBox7.ValueChangedFcn = createCallbackFcn(app, @IMUchange, true);
            app.CheckBox7.Tooltip = {'Only accelerometer and gyro filter.'};
            app.CheckBox7.Text = 'IMU';
            app.CheckBox7.Position = [427 19 41 16];

            % Create CheckBox8
            app.CheckBox8 = uicheckbox(app.Panel3);
            app.CheckBox8.ValueChangedFcn = createCallbackFcn(app, @ExceptChange, true);
            app.CheckBox8.Tooltip = {'There is significant error peak with all filters in set 2, which can be caused by desynchronization of reference data. Check to remove this peak from error estimation.'};
            app.CheckBox8.Text = 'Except Peak';
            app.CheckBox8.Position = [480 19 87 16];

            % Create TextArea
            app.TextArea = uitextarea(app.UIFigure);
            app.TextArea.Position = [358 31 227 93];

            % Create UIAxes2
            app.UIAxes2 = uiaxes(app.UIFigure);
            title(app.UIAxes2, 'Method 1')
            xlabel(app.UIAxes2, 'X')
            ylabel(app.UIAxes2, 'Y')
            app.UIAxes2.GridAlpha = 0.15;
            app.UIAxes2.MinorGridAlpha = 0.25;
            app.UIAxes2.Box = 'on';
            app.UIAxes2.Position = [670 541 258 161];

            % Create UIAxes3
            app.UIAxes3 = uiaxes(app.UIFigure);
            title(app.UIAxes3, 'Method 2')
            xlabel(app.UIAxes3, 'X')
            ylabel(app.UIAxes3, 'Y')
            app.UIAxes3.GridAlpha = 0.15;
            app.UIAxes3.MinorGridAlpha = 0.25;
            app.UIAxes3.Box = 'on';
            app.UIAxes3.Position = [670 377 258 161];

            % Create Button5
            app.Button5 = uibutton(app.UIFigure, 'push');
            app.Button5.ButtonPushedFcn = createCallbackFcn(app, @OptimizeM3, true);
            app.Button5.Position = [259 574 100 22];
            app.Button5.Text = 'Optimize';

            % Create Button6
            app.Button6 = uibutton(app.UIFigure, 'push');
            app.Button6.ButtonPushedFcn = createCallbackFcn(app, @OptimizeM4, true);
            app.Button6.Position = [259 515 100 22];
            app.Button6.Text = 'Optimize';

            % Create Panel4
            app.Panel4 = uipanel(app.UIFigure);
            app.Panel4.AutoResizeChildren = 'off';
            app.Panel4.Title = 'Panel';
            app.Panel4.Position = [392 570 260 50];

            % Create NumericEditField7
            app.NumericEditField7 = uieditfield(app.Panel4, 'numeric');
            app.NumericEditField7.Position = [9 4 61 22];

            % Create NumericEditField8
            app.NumericEditField8 = uieditfield(app.Panel4, 'numeric');
            app.NumericEditField8.Position = [95 4 70 22];

            % Create NumericEditField9
            app.NumericEditField9 = uieditfield(app.Panel4, 'numeric');
            app.NumericEditField9.Position = [190 4 64 22];

            % Create Panel5
            app.Panel5 = uipanel(app.UIFigure);
            app.Panel5.AutoResizeChildren = 'off';
            app.Panel5.Title = 'Panel';
            app.Panel5.Position = [392 511 260 50];

            % Create NumericEditField10
            app.NumericEditField10 = uieditfield(app.Panel5, 'numeric');
            app.NumericEditField10.Position = [9 4 61 22];

            % Create NumericEditField11
            app.NumericEditField11 = uieditfield(app.Panel5, 'numeric');
            app.NumericEditField11.Position = [95 4 70 22];

            % Create NumericEditField12
            app.NumericEditField12 = uieditfield(app.Panel5, 'numeric');
            app.NumericEditField12.Position = [190 4 64 22];

            % Create Label2
            app.Label2 = uilabel(app.UIFigure);
            app.Label2.HorizontalAlignment = 'right';
            app.Label2.VerticalAlignment = 'top';
            app.Label2.Position = [87 579 48 15];
            app.Label2.Text = 'Method3';

            % Create DropDown4
            app.DropDown4 = uidropdown(app.UIFigure);
            app.DropDown4.Items = {'Madgwick', 'JustaFast', 'Valenti', 'Guo', 'MadgwickModif', 'JustaPure', 'JustaLinear', 'JustaConstant', 'YoungSooSuh', 'Wilson'};
            app.DropDown4.ValueChangedFcn = createCallbackFcn(app, @Method3Changed, true);
            app.DropDown4.Position = [142 577 108 20];
            app.DropDown4.Value = 'Valenti';

            % Create Label3
            app.Label3 = uilabel(app.UIFigure);
            app.Label3.HorizontalAlignment = 'right';
            app.Label3.VerticalAlignment = 'top';
            app.Label3.Position = [87 513 48 15];
            app.Label3.Text = 'Method4';

            % Create DropDown5
            app.DropDown5 = uidropdown(app.UIFigure);
            app.DropDown5.Items = {'Madgwick', 'JustaFast', 'Valenti', 'Guo', 'MadgwickModif', 'JustaPure', 'JustaLinear', 'JustaConstant', 'YoungSooSuh', 'Admirall'};
            app.DropDown5.ValueChangedFcn = createCallbackFcn(app, @Method4Changed, true);
            app.DropDown5.Position = [142 511 108 20];
            app.DropDown5.Value = 'Guo';

            % Create UIAxes4
            app.UIAxes4 = uiaxes(app.UIFigure);
            title(app.UIAxes4, 'Method 3')
            xlabel(app.UIAxes4, 'X')
            ylabel(app.UIAxes4, 'Y')
            app.UIAxes4.GridAlpha = 0.15;
            app.UIAxes4.MinorGridAlpha = 0.25;
            app.UIAxes4.Box = 'on';
            app.UIAxes4.Position = [670 205 258 161];

            % Create UIAxes5
            app.UIAxes5 = uiaxes(app.UIFigure);
            title(app.UIAxes5, 'Method 4')
            xlabel(app.UIAxes5, 'X')
            ylabel(app.UIAxes5, 'Y')
            app.UIAxes5.GridAlpha = 0.15;
            app.UIAxes5.MinorGridAlpha = 0.25;
            app.UIAxes5.Box = 'on';
            app.UIAxes5.Position = [670 29 258 161];

            % Create CheckBox3
            app.CheckBox3 = uicheckbox(app.UIFigure);
            app.CheckBox3.Text = '';
            app.CheckBox3.Position = [36 707 14 14];

            % Create CheckBox4
            app.CheckBox4 = uicheckbox(app.UIFigure);
            app.CheckBox4.Text = '';
            app.CheckBox4.Position = [36 642 14 14];

            % Create CheckBox5
            app.CheckBox5 = uicheckbox(app.UIFigure);
            app.CheckBox5.Text = '';
            app.CheckBox5.Position = [36 576 14 14];

            % Create CheckBox6
            app.CheckBox6 = uicheckbox(app.UIFigure);
            app.CheckBox6.Text = '';
            app.CheckBox6.Position = [36 511 14 14];

            % Create IncludeintestLabel
            app.IncludeintestLabel = uilabel(app.UIFigure);
            app.IncludeintestLabel.HorizontalAlignment = 'center';
            app.IncludeintestLabel.Position = [21 724 44 46];
            app.IncludeintestLabel.Text = {'Include'; 'in test'};

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = TestApplication

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end

