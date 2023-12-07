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



class grasp_selector:

    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        target_frame = "world"
        input_topic = "object_pose"
        output_topic = "desired_eef_pose"
        class __Offset:
            x = -0.02
            y = 0.0
            z = 0.0
        offset = __Offset()



    class ParamListener:
        def __init__(self, node, prefix=""):
            node.declare_parameter('my_parameter', 'world')
            self.prefix_ = prefix
            self.params_ = grasp_selector.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("grasp_selector." + prefix)

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
                if param.name == self.prefix_ + "target_frame":
                    validation_result = ParameterValidators.not_empty(param)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.target_frame = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "input_topic":
                    validation_result = ParameterValidators.not_empty(param)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.input_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "output_topic":
                    validation_result = ParameterValidators.not_empty(param)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.output_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "offset.x":
                    updated_params.offset.x = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "offset.y":
                    updated_params.offset.y = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "offset.z":
                    updated_params.offset.z = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))



            updated_params.stamp_ = self.clock_.now()
            self.update_internal_params(updated_params)
            return SetParametersResult(successful=True)

        def update_internal_params(self, updated_params):
            self.params_ = updated_params

        def declare_params(self):
            updated_params = self.get_params()
            # declare all parameters and give default values to non-required ones
            if not self.node_.has_parameter(self.prefix_ + "target_frame"):
                descriptor = ParameterDescriptor(description="Name of TF frame that grasp should be expressed in.", read_only = True)
                parameter = updated_params.target_frame
                self.node_.declare_parameter(self.prefix_ + "target_frame", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "input_topic"):
                descriptor = ParameterDescriptor(description="Name of PoseStamped topic with the object pose.", read_only = True)
                parameter = updated_params.input_topic
                self.node_.declare_parameter(self.prefix_ + "input_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "output_topic"):
                descriptor = ParameterDescriptor(description="Name of PoseStamped topic to publish with grasp.", read_only = True)
                parameter = updated_params.output_topic
                self.node_.declare_parameter(self.prefix_ + "output_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "offset.x"):
                descriptor = ParameterDescriptor(description="X-axis offset from object_pose to desired_eef_pose.", read_only = False)
                parameter = updated_params.offset.x
                self.node_.declare_parameter(self.prefix_ + "offset.x", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "offset.y"):
                descriptor = ParameterDescriptor(description="Y-axis offset from object_pose to desired_eef_pose.", read_only = False)
                parameter = updated_params.offset.y
                self.node_.declare_parameter(self.prefix_ + "offset.y", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "offset.z"):
                descriptor = ParameterDescriptor(description="Z-axis offset from object_pose to desired_eef_pose.", read_only = False)
                parameter = updated_params.offset.z
                self.node_.declare_parameter(self.prefix_ + "offset.z", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "target_frame")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.not_empty(param)
            if validation_result:
                raise InvalidParameterValueException('Invalid value set during initialization for parameter target_frame: ' + validation_result)
            updated_params.target_frame = param.value
            param = self.node_.get_parameter(self.prefix_ + "input_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.not_empty(param)
            if validation_result:
                raise InvalidParameterValueException('Invalid value set during initialization for parameter input_topic: ' + validation_result)
            updated_params.input_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "output_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.not_empty(param)
            if validation_result:
                raise InvalidParameterValueException('Invalid value set during initialization for parameter output_topic: ' + validation_result)
            updated_params.output_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "offset.x")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.offset.x = param.value
            param = self.node_.get_parameter(self.prefix_ + "offset.y")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.offset.y = param.value
            param = self.node_.get_parameter(self.prefix_ + "offset.z")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.offset.z = param.value


            self.update_internal_params(updated_params)