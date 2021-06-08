classdef status_arm < robotics.ros.Message
    %status_arm MATLAB implementation of custom_msg/status_arm
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2021 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'custom_msg/status_arm' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'c3c5e83e349a8305c77eff5da188ae4a' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Dependent)
        Junta
        PulsosSetpoint
        PulsosContados
        PulsosErro
        OutputP
        OutputI
        OutputD
        OutputPID
        LoopTime
        IsDone
    end
    
    properties (Constant, Hidden)
        PropertyList = {'IsDone', 'Junta', 'LoopTime', 'OutputD', 'OutputI', 'OutputP', 'OutputPID', 'PulsosContados', 'PulsosErro', 'PulsosSetpoint'} % List of non-constant message properties
        ROSPropertyList = {'IsDone', 'junta', 'loop_time', 'output_D', 'output_I', 'output_P', 'output_PID', 'pulsos_contados', 'pulsos_erro', 'pulsos_setpoint'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = status_arm(msg)
            %status_arm Construct the message object status_arm
            import com.mathworks.toolbox.robotics.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('robotics:ros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('robotics:ros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('robotics:ros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function junta = get.Junta(obj)
            %get.Junta Get the value for property Junta
            junta = char(obj.JavaMessage.getJunta);
        end
        
        function set.Junta(obj, junta)
            %set.Junta Set the value for property Junta
            junta = convertStringsToChars(junta);
            
            validateattributes(junta, {'char', 'string'}, {}, 'status_arm', 'Junta');
            
            obj.JavaMessage.setJunta(junta);
        end
        
        function pulsossetpoint = get.PulsosSetpoint(obj)
            %get.PulsosSetpoint Get the value for property PulsosSetpoint
            pulsossetpoint = single(obj.JavaMessage.getPulsosSetpoint);
        end
        
        function set.PulsosSetpoint(obj, pulsossetpoint)
            %set.PulsosSetpoint Set the value for property PulsosSetpoint
            validateattributes(pulsossetpoint, {'numeric'}, {'nonempty', 'scalar'}, 'status_arm', 'PulsosSetpoint');
            
            obj.JavaMessage.setPulsosSetpoint(pulsossetpoint);
        end
        
        function pulsoscontados = get.PulsosContados(obj)
            %get.PulsosContados Get the value for property PulsosContados
            pulsoscontados = single(obj.JavaMessage.getPulsosContados);
        end
        
        function set.PulsosContados(obj, pulsoscontados)
            %set.PulsosContados Set the value for property PulsosContados
            validateattributes(pulsoscontados, {'numeric'}, {'nonempty', 'scalar'}, 'status_arm', 'PulsosContados');
            
            obj.JavaMessage.setPulsosContados(pulsoscontados);
        end
        
        function pulsoserro = get.PulsosErro(obj)
            %get.PulsosErro Get the value for property PulsosErro
            pulsoserro = single(obj.JavaMessage.getPulsosErro);
        end
        
        function set.PulsosErro(obj, pulsoserro)
            %set.PulsosErro Set the value for property PulsosErro
            validateattributes(pulsoserro, {'numeric'}, {'nonempty', 'scalar'}, 'status_arm', 'PulsosErro');
            
            obj.JavaMessage.setPulsosErro(pulsoserro);
        end
        
        function outputp = get.OutputP(obj)
            %get.OutputP Get the value for property OutputP
            outputp = single(obj.JavaMessage.getOutputP);
        end
        
        function set.OutputP(obj, outputp)
            %set.OutputP Set the value for property OutputP
            validateattributes(outputp, {'numeric'}, {'nonempty', 'scalar'}, 'status_arm', 'OutputP');
            
            obj.JavaMessage.setOutputP(outputp);
        end
        
        function outputi = get.OutputI(obj)
            %get.OutputI Get the value for property OutputI
            outputi = single(obj.JavaMessage.getOutputI);
        end
        
        function set.OutputI(obj, outputi)
            %set.OutputI Set the value for property OutputI
            validateattributes(outputi, {'numeric'}, {'nonempty', 'scalar'}, 'status_arm', 'OutputI');
            
            obj.JavaMessage.setOutputI(outputi);
        end
        
        function outputd = get.OutputD(obj)
            %get.OutputD Get the value for property OutputD
            outputd = single(obj.JavaMessage.getOutputD);
        end
        
        function set.OutputD(obj, outputd)
            %set.OutputD Set the value for property OutputD
            validateattributes(outputd, {'numeric'}, {'nonempty', 'scalar'}, 'status_arm', 'OutputD');
            
            obj.JavaMessage.setOutputD(outputd);
        end
        
        function outputpid = get.OutputPID(obj)
            %get.OutputPID Get the value for property OutputPID
            outputpid = single(obj.JavaMessage.getOutputPID);
        end
        
        function set.OutputPID(obj, outputpid)
            %set.OutputPID Set the value for property OutputPID
            validateattributes(outputpid, {'numeric'}, {'nonempty', 'scalar'}, 'status_arm', 'OutputPID');
            
            obj.JavaMessage.setOutputPID(outputpid);
        end
        
        function looptime = get.LoopTime(obj)
            %get.LoopTime Get the value for property LoopTime
            looptime = single(obj.JavaMessage.getLoopTime);
        end
        
        function set.LoopTime(obj, looptime)
            %set.LoopTime Set the value for property LoopTime
            validateattributes(looptime, {'numeric'}, {'nonempty', 'scalar'}, 'status_arm', 'LoopTime');
            
            obj.JavaMessage.setLoopTime(looptime);
        end
        
        function isdone = get.IsDone(obj)
            %get.IsDone Get the value for property IsDone
            isdone = logical(obj.JavaMessage.getIsDone);
        end
        
        function set.IsDone(obj, isdone)
            %set.IsDone Set the value for property IsDone
            validateattributes(isdone, {'logical', 'numeric'}, {'nonempty', 'scalar'}, 'status_arm', 'IsDone');
            
            obj.JavaMessage.setIsDone(isdone);
        end
    end
    
    methods (Access = protected)
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.Junta = obj.Junta;
            cpObj.PulsosSetpoint = obj.PulsosSetpoint;
            cpObj.PulsosContados = obj.PulsosContados;
            cpObj.PulsosErro = obj.PulsosErro;
            cpObj.OutputP = obj.OutputP;
            cpObj.OutputI = obj.OutputI;
            cpObj.OutputD = obj.OutputD;
            cpObj.OutputPID = obj.OutputPID;
            cpObj.LoopTime = obj.LoopTime;
            cpObj.IsDone = obj.IsDone;
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Junta = strObj.Junta;
            obj.PulsosSetpoint = strObj.PulsosSetpoint;
            obj.PulsosContados = strObj.PulsosContados;
            obj.PulsosErro = strObj.PulsosErro;
            obj.OutputP = strObj.OutputP;
            obj.OutputI = strObj.OutputI;
            obj.OutputD = strObj.OutputD;
            obj.OutputPID = strObj.OutputPID;
            obj.LoopTime = strObj.LoopTime;
            obj.IsDone = strObj.IsDone;
        end
    end
    
    methods (Access = ?robotics.ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.Junta = obj.Junta;
            strObj.PulsosSetpoint = obj.PulsosSetpoint;
            strObj.PulsosContados = obj.PulsosContados;
            strObj.PulsosErro = obj.PulsosErro;
            strObj.OutputP = obj.OutputP;
            strObj.OutputI = obj.OutputI;
            strObj.OutputD = obj.OutputD;
            strObj.OutputPID = obj.OutputPID;
            strObj.LoopTime = obj.LoopTime;
            strObj.IsDone = obj.IsDone;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.custom_msg.status_arm.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.custom_msg.status_arm;
            obj.reload(strObj);
        end
    end
end