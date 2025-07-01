#pragma message("#include \"phidgets_spatial_parameters.hpp\" is deprecated. Use #include <husarion_ugv_hardware_interfaces/phidgets_spatial_parameters.hpp> instead.")
// auto-generated DO NOT EDIT

#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/logger.hpp>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <parameter_traits/parameter_traits.hpp>

#include <rsl/static_string.hpp>
#include <rsl/static_vector.hpp>
#include <rsl/parameter_validators.hpp>



namespace phidgets_spatial {

// Use validators from RSL
using rsl::unique;
using rsl::subset_of;
using rsl::fixed_size;
using rsl::size_gt;
using rsl::size_lt;
using rsl::not_empty;
using rsl::element_bounds;
using rsl::lower_element_bounds;
using rsl::upper_element_bounds;
using rsl::bounds;
using rsl::lt;
using rsl::gt;
using rsl::lt_eq;
using rsl::gt_eq;
using rsl::one_of;
using rsl::to_parameter_result_msg;

// temporarily needed for backwards compatibility for custom validators
using namespace parameter_traits;

template <typename T>
[[nodiscard]] auto to_parameter_value(T value) {
    return rclcpp::ParameterValue(value);
}

template <size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticString<capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_string(value));
}

template <typename T, size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticVector<T, capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_vector(value));
}
    struct Params {
        int64_t serial = -1;
        int64_t hub_port = 0;
        bool heating_enabled = false;
        int64_t time_resynchronization_interval_ms = 5000;
        int64_t data_interval_ms = 8;
        int64_t callback_delta_epsilon_ms = 1;
        double cc_mag_field = 0.0;
        double cc_offset0 = 0.0;
        double cc_offset1 = 0.0;
        double cc_offset2 = 0.0;
        double cc_gain0 = 0.0;
        double cc_gain1 = 0.0;
        double cc_gain2 = 0.0;
        double cc_t0 = 0.0;
        double cc_t1 = 0.0;
        double cc_t2 = 0.0;
        double cc_t3 = 0.0;
        double cc_t4 = 0.0;
        double cc_t5 = 0.0;
        bool use_mag = false;
        double gain = 0.1;
        double zeta = 0.1;
        double mag_bias_x = 0.0;
        double mag_bias_y = 0.0;
        double mag_bias_z = 0.0;
        bool stateless = false;
        bool remove_gravity_vector = false;
        // for detecting if the parameter struct has been updated
        rclcpp::Time __stamp;
    };
    struct StackParams {
        int64_t serial = -1;
        int64_t hub_port = 0;
        bool heating_enabled = false;
        int64_t time_resynchronization_interval_ms = 5000;
        int64_t data_interval_ms = 8;
        int64_t callback_delta_epsilon_ms = 1;
        double cc_mag_field = 0.0;
        double cc_offset0 = 0.0;
        double cc_offset1 = 0.0;
        double cc_offset2 = 0.0;
        double cc_gain0 = 0.0;
        double cc_gain1 = 0.0;
        double cc_gain2 = 0.0;
        double cc_t0 = 0.0;
        double cc_t1 = 0.0;
        double cc_t2 = 0.0;
        double cc_t3 = 0.0;
        double cc_t4 = 0.0;
        double cc_t5 = 0.0;
        bool use_mag = false;
        double gain = 0.1;
        double zeta = 0.1;
        double mag_bias_x = 0.0;
        double mag_bias_y = 0.0;
        double mag_bias_z = 0.0;
        bool stateless = false;
        bool remove_gravity_vector = false;
    };

  class ParamListener{
  public:
    // throws rclcpp::exceptions::InvalidParameterValueException on initialization if invalid parameter are loaded
    ParamListener(rclcpp::Node::SharedPtr node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  std::string const& prefix = "")
    : ParamListener(parameters_interface, rclcpp::get_logger("phidgets_spatial"), prefix) {
      RCLCPP_DEBUG(logger_, "ParameterListener: Not using node logger, recommend using other constructors to use a node logger");
    }

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  rclcpp::Logger logger, std::string const& prefix = "") {
      logger_ = std::move(logger);
      prefix_ = prefix;
      if (!prefix_.empty() && prefix_.back() != '.') {
        prefix_ += ".";
      }

      parameters_interface_ = parameters_interface;
      declare_params();
      auto update_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters){return this->update(parameters);};
      handle_ = parameters_interface_->add_on_set_parameters_callback(update_param_cb);
      clock_ = rclcpp::Clock();
    }

    Params get_params() const{
      std::lock_guard<std::mutex> lock(mutex_);
      return params_;
    }

    bool try_get_params(Params & params_in) const {
      if (mutex_.try_lock()) {
        if (const bool is_old = params_in.__stamp != params_.__stamp; is_old) {
          params_in = params_;
        }
        mutex_.unlock();
        return true;
      }
      return false;
    }

    bool is_old(Params const& other) const {
      std::lock_guard<std::mutex> lock(mutex_);
      return params_.__stamp != other.__stamp;
    }

    StackParams get_stack_params() {
      Params params = get_params();
      StackParams output;
      output.serial = params.serial;
      output.hub_port = params.hub_port;
      output.heating_enabled = params.heating_enabled;
      output.time_resynchronization_interval_ms = params.time_resynchronization_interval_ms;
      output.data_interval_ms = params.data_interval_ms;
      output.callback_delta_epsilon_ms = params.callback_delta_epsilon_ms;
      output.cc_mag_field = params.cc_mag_field;
      output.cc_offset0 = params.cc_offset0;
      output.cc_offset1 = params.cc_offset1;
      output.cc_offset2 = params.cc_offset2;
      output.cc_gain0 = params.cc_gain0;
      output.cc_gain1 = params.cc_gain1;
      output.cc_gain2 = params.cc_gain2;
      output.cc_t0 = params.cc_t0;
      output.cc_t1 = params.cc_t1;
      output.cc_t2 = params.cc_t2;
      output.cc_t3 = params.cc_t3;
      output.cc_t4 = params.cc_t4;
      output.cc_t5 = params.cc_t5;
      output.use_mag = params.use_mag;
      output.gain = params.gain;
      output.zeta = params.zeta;
      output.mag_bias_x = params.mag_bias_x;
      output.mag_bias_y = params.mag_bias_y;
      output.mag_bias_z = params.mag_bias_z;
      output.stateless = params.stateless;
      output.remove_gravity_vector = params.remove_gravity_vector;

      return output;
    }

    void refresh_dynamic_parameters() {
      auto updated_params = get_params();
      // TODO remove any destroyed dynamic parameters

      // declare any new dynamic parameters
      rclcpp::Parameter param;

    }

    rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters) {
      auto updated_params = get_params();

      for (const auto &param: parameters) {
        if (param.get_name() == (prefix_ + "serial")) {
            updated_params.serial = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "hub_port")) {
            updated_params.hub_port = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "heating_enabled")) {
            updated_params.heating_enabled = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "time_resynchronization_interval_ms")) {
            updated_params.time_resynchronization_interval_ms = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "data_interval_ms")) {
            updated_params.data_interval_ms = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "callback_delta_epsilon_ms")) {
            updated_params.callback_delta_epsilon_ms = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cc_mag_field")) {
            updated_params.cc_mag_field = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cc_offset0")) {
            updated_params.cc_offset0 = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cc_offset1")) {
            updated_params.cc_offset1 = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cc_offset2")) {
            updated_params.cc_offset2 = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cc_gain0")) {
            updated_params.cc_gain0 = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cc_gain1")) {
            updated_params.cc_gain1 = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cc_gain2")) {
            updated_params.cc_gain2 = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cc_t0")) {
            updated_params.cc_t0 = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cc_t1")) {
            updated_params.cc_t1 = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cc_t2")) {
            updated_params.cc_t2 = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cc_t3")) {
            updated_params.cc_t3 = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cc_t4")) {
            updated_params.cc_t4 = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cc_t5")) {
            updated_params.cc_t5 = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "use_mag")) {
            updated_params.use_mag = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "gain")) {
            updated_params.gain = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "zeta")) {
            updated_params.zeta = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "mag_bias_x")) {
            updated_params.mag_bias_x = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "mag_bias_y")) {
            updated_params.mag_bias_y = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "mag_bias_z")) {
            updated_params.mag_bias_z = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "stateless")) {
            updated_params.stateless = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "remove_gravity_vector")) {
            updated_params.remove_gravity_vector = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
      }

      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
      return rsl::to_parameter_result_msg({});
    }

    void declare_params(){
      auto updated_params = get_params();
      // declare all parameters and give default values to non-required ones
      if (!parameters_interface_->has_parameter(prefix_ + "serial")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The serial number of the phidgets spatial to connect to. If -1 (the default), connects to any spatial phidget that can be found.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.serial);
          parameters_interface_->declare_parameter(prefix_ + "serial", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "hub_port")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The phidgets VINT hub port to connect to. Only used if the spatial phidget is connected to a VINT hub. Defaults to 0.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.hub_port);
          parameters_interface_->declare_parameter(prefix_ + "hub_port", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "heating_enabled")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Use the internal heating element; Just available on MOT0109 onwards. Do not set this parameter for older versions.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.heating_enabled);
          parameters_interface_->declare_parameter(prefix_ + "heating_enabled", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "time_resynchronization_interval_ms")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The number of milliseconds to wait between resynchronizing the time on the Phidgets spatial with the local time. Larger values have less 'jumps', but will have more timestamp drift. Setting this to 0 disables resynchronization. Defaults to 5000 ms.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.time_resynchronization_interval_ms);
          parameters_interface_->declare_parameter(prefix_ + "time_resynchronization_interval_ms", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "data_interval_ms")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The number of milliseconds between acquisitions of data on the device (allowed values are dependent on the device). Defaults to 8 ms.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.data_interval_ms);
          parameters_interface_->declare_parameter(prefix_ + "data_interval_ms", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "callback_delta_epsilon_ms")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The number of milliseconds epsilon allowed between callbacks when attempting to resynchronize the time. If this is set to 1, then a difference of data_interval_ms plus or minus 1 millisecond will be considered viable for resynchronization. Higher values give the code more leeway to resynchronize, at the cost of potentially getting bad resynchronizations sometimes. Lower values can give better results, but can also result in never resynchronizing. Must be less than data_interval_ms. Defaults to 1 ms.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.callback_delta_epsilon_ms);
          parameters_interface_->declare_parameter(prefix_ + "callback_delta_epsilon_ms", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cc_mag_field")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Ambient magnetic field calibration value; see device's user guide for information on how to calibrate.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.cc_mag_field);
          parameters_interface_->declare_parameter(prefix_ + "cc_mag_field", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cc_offset0")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Calibration offset value 0; see device's user guide for information on how to calibrate.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.cc_offset0);
          parameters_interface_->declare_parameter(prefix_ + "cc_offset0", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cc_offset1")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Calibration offset value 1; see device's user guide for information on how to calibrate.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.cc_offset1);
          parameters_interface_->declare_parameter(prefix_ + "cc_offset1", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cc_offset2")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Calibration offset value 2; see device's user guide for information on how to calibrate.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.cc_offset2);
          parameters_interface_->declare_parameter(prefix_ + "cc_offset2", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cc_gain0")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Gain offset value 0; see device's user guide for information on how to calibrate.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.cc_gain0);
          parameters_interface_->declare_parameter(prefix_ + "cc_gain0", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cc_gain1")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = " Gain offset value 1; see device's user guide for information on how to calibrate.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.cc_gain1);
          parameters_interface_->declare_parameter(prefix_ + "cc_gain1", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cc_gain2")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Gain offset value 2; see device's user guide for information on how to calibrate.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.cc_gain2);
          parameters_interface_->declare_parameter(prefix_ + "cc_gain2", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cc_t0")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "T offset value 0; see device's user guide for information on how to calibrate.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.cc_t0);
          parameters_interface_->declare_parameter(prefix_ + "cc_t0", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cc_t1")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "T offset value 1; see device's user guide for information on how to calibrate.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.cc_t1);
          parameters_interface_->declare_parameter(prefix_ + "cc_t1", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cc_t2")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "T offset value 2; see device's user guide for information on how to calibrate.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.cc_t2);
          parameters_interface_->declare_parameter(prefix_ + "cc_t2", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cc_t3")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "T offset value 3; see device's user guide for information on how to calibrate.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.cc_t3);
          parameters_interface_->declare_parameter(prefix_ + "cc_t3", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cc_t4")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "T offset value 4; see device's user guide for information on how to calibrate.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.cc_t4);
          parameters_interface_->declare_parameter(prefix_ + "cc_t4", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cc_t5")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "T offset value 5; see device's user guide for information on how to calibrate.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.cc_t5);
          parameters_interface_->declare_parameter(prefix_ + "cc_t5", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "use_mag")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Use magnitude to calculate orientation.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.use_mag);
          parameters_interface_->declare_parameter(prefix_ + "use_mag", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "gain")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Gain of the filter. Higher values lead to faster convergence but more noise. Lower values lead to slower convergence but smoother signal.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.gain);
          parameters_interface_->declare_parameter(prefix_ + "gain", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "zeta")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Gyro drift gain (approx. rad/s).";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.zeta);
          parameters_interface_->declare_parameter(prefix_ + "zeta", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "mag_bias_x")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Magnetometer bias (hard iron correction), x component.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.mag_bias_x);
          parameters_interface_->declare_parameter(prefix_ + "mag_bias_x", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "mag_bias_y")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Magnetometer bias (hard iron correction), y component.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.mag_bias_y);
          parameters_interface_->declare_parameter(prefix_ + "mag_bias_y", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "mag_bias_z")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Magnetometer bias (hard iron correction), z component.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.mag_bias_z);
          parameters_interface_->declare_parameter(prefix_ + "mag_bias_z", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "stateless")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Use stateless to compute orientation on every data callback without prediction based on previous measurements.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.stateless);
          parameters_interface_->declare_parameter(prefix_ + "stateless", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "remove_gravity_vector")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The gravity vector is kept in the IMU message.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.remove_gravity_vector);
          parameters_interface_->declare_parameter(prefix_ + "remove_gravity_vector", parameter, descriptor);
      }
      // get parameters and fill struct fields
      rclcpp::Parameter param;
      param = parameters_interface_->get_parameter(prefix_ + "serial");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.serial = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "hub_port");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.hub_port = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "heating_enabled");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.heating_enabled = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "time_resynchronization_interval_ms");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.time_resynchronization_interval_ms = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "data_interval_ms");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.data_interval_ms = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "callback_delta_epsilon_ms");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.callback_delta_epsilon_ms = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "cc_mag_field");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cc_mag_field = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "cc_offset0");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cc_offset0 = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "cc_offset1");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cc_offset1 = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "cc_offset2");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cc_offset2 = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "cc_gain0");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cc_gain0 = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "cc_gain1");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cc_gain1 = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "cc_gain2");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cc_gain2 = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "cc_t0");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cc_t0 = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "cc_t1");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cc_t1 = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "cc_t2");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cc_t2 = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "cc_t3");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cc_t3 = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "cc_t4");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cc_t4 = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "cc_t5");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cc_t5 = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "use_mag");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.use_mag = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "gain");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.gain = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "zeta");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.zeta = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "mag_bias_x");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.mag_bias_x = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "mag_bias_y");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.mag_bias_y = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "mag_bias_z");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.mag_bias_z = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "stateless");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.stateless = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "remove_gravity_vector");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.remove_gravity_vector = param.as_bool();


      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
    }

    private:
      void update_internal_params(Params updated_params) {
        std::lock_guard<std::mutex> lock(mutex_);
        params_ = std::move(updated_params);
      }

      std::string prefix_;
      Params params_;
      rclcpp::Clock clock_;
      std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;
      std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;

      // rclcpp::Logger cannot be default-constructed
      // so we must provide a initialization here even though
      // every one of our constructors initializes logger_
      rclcpp::Logger logger_ = rclcpp::get_logger("phidgets_spatial");
      std::mutex mutable mutex_;
  };

} // namespace phidgets_spatial
