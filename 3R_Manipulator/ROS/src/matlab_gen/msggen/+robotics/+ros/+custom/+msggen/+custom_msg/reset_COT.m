classdef reset_COT < robotics.ros.Message
    %reset_COT MATLAB implementation of custom_msg/reset_COT
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2021 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'custom_msg/reset_COT' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '8be9f92adaf46f35e0791e11db65f1f5' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Dependent)
        ResetCOT
    end
    
    properties (Constant, Hidden)
        PropertyList = {'ResetCOT'} % List of non-constant message properties
        ROSPropertyList = {'reset_COT'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = reset_COT(msg)
            %reset_COT Construct the message object reset_COT
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
        
        function resetcot = get.ResetCOT(obj)
            %get.ResetCOT Get the value for property ResetCOT
            resetcot = logical(obj.JavaMessage.getResetCOT);
        end
        
        function set.ResetCOT(obj, resetcot)
            %set.ResetCOT Set the value for property ResetCOT
            validateattributes(resetcot, {'logical', 'numeric'}, {'nonempty', 'scalar'}, 'reset_COT', 'ResetCOT');
            
            obj.JavaMessage.setResetCOT(resetcot);
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
            cpObj.ResetCOT = obj.ResetCOT;
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.ResetCOT = strObj.ResetCOT;
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
            
            strObj.ResetCOT = obj.ResetCOT;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.custom_msg.reset_COT.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.custom_msg.reset_COT;
            obj.reload(strObj);
        end
    end
end
