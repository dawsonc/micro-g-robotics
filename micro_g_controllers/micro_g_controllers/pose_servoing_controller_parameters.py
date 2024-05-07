# flake8: noqa

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



class pose_servoing:

    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        control_update_rate = 20.0
        moving_time = 0.5
        kp = 0.6
        replanning_attempts = 5
        timeout = 1.0



    class ParamListener:
        def __init__(self, node, prefix=""):
            self.prefix_ = prefix
            self.params_ = pose_servoing.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("pose_servoing." + prefix)

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
                if param.name == self.prefix_ + "control_update_rate":
                    validation_result = ParameterValidators.gt_eq(param, 0.0001)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.control_update_rate = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "moving_time":
                    updated_params.moving_time = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "kp":
                    validation_result = ParameterValidators.gt_eq(param, 0.0001)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.kp = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "replanning_attempts":
                    validation_result = ParameterValidators.gt_eq(param, 0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.replanning_attempts = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "timeout":
                    validation_result = ParameterValidators.gt_eq(param, 0.0001)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.timeout = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))



            updated_params.stamp_ = self.clock_.now()
            self.update_internal_params(updated_params)
            return SetParametersResult(successful=True)

        def update_internal_params(self, updated_params):
            self.params_ = updated_params

        def declare_params(self):
            updated_params = self.get_params()
            # declare all parameters and give default values to non-required ones
            if not self.node_.has_parameter(self.prefix_ + "control_update_rate"):
                descriptor = ParameterDescriptor(description="Control update rate in Hz", read_only = True)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0001
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.control_update_rate
                self.node_.declare_parameter(self.prefix_ + "control_update_rate", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "moving_time"):
                descriptor = ParameterDescriptor(description="Moving time in seconds", read_only = False)
                parameter = updated_params.moving_time
                self.node_.declare_parameter(self.prefix_ + "moving_time", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "kp"):
                descriptor = ParameterDescriptor(description="Proportional gain for joint tracking controller", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0001
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.kp
                self.node_.declare_parameter(self.prefix_ + "kp", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "replanning_attempts"):
                descriptor = ParameterDescriptor(description="Number of times to attempt re-planning", read_only = False)
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 2**31-1
                parameter = updated_params.replanning_attempts
                self.node_.declare_parameter(self.prefix_ + "replanning_attempts", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "timeout"):
                descriptor = ParameterDescriptor(description="Wait this long after getting a target before moving home (s)", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0001
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.timeout
                self.node_.declare_parameter(self.prefix_ + "timeout", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "control_update_rate")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0001)
            if validation_result:
                raise InvalidParameterValueException('control_update_rate',param.value, 'Invalid value set during initialization for parameter control_update_rate: ' + validation_result)
            updated_params.control_update_rate = param.value
            param = self.node_.get_parameter(self.prefix_ + "moving_time")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.moving_time = param.value
            param = self.node_.get_parameter(self.prefix_ + "kp")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0001)
            if validation_result:
                raise InvalidParameterValueException('kp',param.value, 'Invalid value set during initialization for parameter kp: ' + validation_result)
            updated_params.kp = param.value
            param = self.node_.get_parameter(self.prefix_ + "replanning_attempts")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0)
            if validation_result:
                raise InvalidParameterValueException('replanning_attempts',param.value, 'Invalid value set during initialization for parameter replanning_attempts: ' + validation_result)
            updated_params.replanning_attempts = param.value
            param = self.node_.get_parameter(self.prefix_ + "timeout")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0001)
            if validation_result:
                raise InvalidParameterValueException('timeout',param.value, 'Invalid value set during initialization for parameter timeout: ' + validation_result)
            updated_params.timeout = param.value


            self.update_internal_params(updated_params)
