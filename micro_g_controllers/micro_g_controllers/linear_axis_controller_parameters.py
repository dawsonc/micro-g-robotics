# auto-generated DO NOT EDIT

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import FloatingPointRange, IntegerRange
from rclpy.clock import Clock
from rclpy.exceptions import InvalidParameterValueException
from rclpy.time import Time
import copy
import rclpy
from generate_parameter_library_py.python_validators import ParameterValidators



class linear_axis_controller:

    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        input_topic = "/px100/desired_eef_pose"
        control_frequency = 20.0
        kp = 3.0
        max_speed = 0.75
        command_timeout = 1.0
        class __PositionLimits:
            min = 0
            max = 2500
        position_limits = __PositionLimits()



    class ParamListener:
        def __init__(self, node, prefix=""):
            node.declare_parameter('my_parameter', 'world')
            self.prefix_ = prefix
            self.params_ = linear_axis_controller.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("linear_axis_controller." + prefix)

            self.declare_params()

            self.node_.add_on_set_parameters_callback(self.update)
            self.clock_ = Clock()

        def get_params(self):
            tmp = self.params_.stamp_
            self.params_.stamp_ = None
            paramCopy = copy.deepcopy(self.params_)
            paramCopy.stamp_ = tmp
            self.params_.stamp_ = tmp
            return paramCopy

        def is_old(self, other_param):
            return self.params_.stamp_ != other_param.stamp_

        def refresh_dynamic_parameters(self):
            updated_params = self.get_params()
            # TODO remove any destroyed dynamic parameters

            # declare any new dynamic parameters


        def update(self, parameters):
            updated_params = self.get_params()

            for param in parameters:
                if param.name == self.prefix_ + "input_topic":
                    validation_result = ParameterValidators.not_empty(param)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.input_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "control_frequency":
                    validation_result = ParameterValidators.gt_eq(param, 0.0001)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.control_frequency = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "kp":
                    validation_result = ParameterValidators.gt_eq(param, 0.0001)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.kp = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "max_speed":
                    validation_result = ParameterValidators.gt_eq(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.max_speed = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "position_limits.min":
                    updated_params.position_limits.min = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "position_limits.max":
                    updated_params.position_limits.max = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "command_timeout":
                    validation_result = ParameterValidators.gt_eq(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.command_timeout = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))



            updated_params.stamp_ = self.clock_.now()
            self.update_internal_params(updated_params)
            return SetParametersResult(successful=True)

        def update_internal_params(self, updated_params):
            self.params_ = updated_params

        def declare_params(self):
            updated_params = self.get_params()
            # declare all parameters and give default values to non-required ones
            if not self.node_.has_parameter(self.prefix_ + "input_topic"):
                descriptor = ParameterDescriptor(description="Name of PoseStamped topic with the object pose.", read_only = True)
                parameter = updated_params.input_topic
                self.node_.declare_parameter(self.prefix_ + "input_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "control_frequency"):
                descriptor = ParameterDescriptor(description="Control rate (Hz).", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0001
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.control_frequency
                self.node_.declare_parameter(self.prefix_ + "control_frequency", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "kp"):
                descriptor = ParameterDescriptor(description="Proportional gain for linear axis controller", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0001
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.kp
                self.node_.declare_parameter(self.prefix_ + "kp", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "max_speed"):
                descriptor = ParameterDescriptor(description="Max speed (m/s)", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.max_speed
                self.node_.declare_parameter(self.prefix_ + "max_speed", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "position_limits.min"):
                descriptor = ParameterDescriptor(description="Min motor position (pulses)", read_only = False)
                parameter = updated_params.position_limits.min
                self.node_.declare_parameter(self.prefix_ + "position_limits.min", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "position_limits.max"):
                descriptor = ParameterDescriptor(description="Max motor position from the home position (pulses)", read_only = False)
                parameter = updated_params.position_limits.max
                self.node_.declare_parameter(self.prefix_ + "position_limits.max", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "command_timeout"):
                descriptor = ParameterDescriptor(description="Max duration before a command is considered stale and the controller stops", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.command_timeout
                self.node_.declare_parameter(self.prefix_ + "command_timeout", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "input_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.not_empty(param)
            if validation_result:
                raise InvalidParameterValueException('Invalid value set during initialization for parameter input_topic: ' + validation_result)
            updated_params.input_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "control_frequency")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0001)
            if validation_result:
                raise InvalidParameterValueException('Invalid value set during initialization for parameter control_frequency: ' + validation_result)
            updated_params.control_frequency = param.value
            param = self.node_.get_parameter(self.prefix_ + "kp")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0001)
            if validation_result:
                raise InvalidParameterValueException('Invalid value set during initialization for parameter kp: ' + validation_result)
            updated_params.kp = param.value
            param = self.node_.get_parameter(self.prefix_ + "max_speed")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('Invalid value set during initialization for parameter max_speed: ' + validation_result)
            updated_params.max_speed = param.value
            param = self.node_.get_parameter(self.prefix_ + "position_limits.min")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.position_limits.min = param.value
            param = self.node_.get_parameter(self.prefix_ + "position_limits.max")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.position_limits.max = param.value
            param = self.node_.get_parameter(self.prefix_ + "command_timeout")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('Invalid value set during initialization for parameter command_timeout: ' + validation_result)
            updated_params.command_timeout = param.value


            self.update_internal_params(updated_params)