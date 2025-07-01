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



namespace battery {

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
        double battery_timeout = 1.0;
        struct Adc {
            std::string device0 = "/sys/bus/iio/devices/iio:device0";
            std::string device1 = "/sys/bus/iio/devices/iio:device1";
            struct MaWindowLen {
                int64_t charge = 10;
                int64_t temp = 10;
            } ma_window_len;
        } adc;
        struct MaWindowLen {
            int64_t voltage = 10;
            int64_t current = 10;
        } ma_window_len;
        struct Roboteq {
            double driver_state_timeout = 0.2;
        } roboteq;
        // for detecting if the parameter struct has been updated
        rclcpp::Time __stamp;
    };
    struct StackParams {
        double battery_timeout = 1.0;
        struct Adc {
            struct MaWindowLen {
                int64_t charge = 10;
                int64_t temp = 10;
            } ma_window_len;
        } adc;
        struct MaWindowLen {
            int64_t voltage = 10;
            int64_t current = 10;
        } ma_window_len;
        struct Roboteq {
            double driver_state_timeout = 0.2;
        } roboteq;
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
    : ParamListener(parameters_interface, rclcpp::get_logger("battery"), prefix) {
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
      output.adc.ma_window_len.charge = params.adc.ma_window_len.charge;
      output.adc.ma_window_len.temp = params.adc.ma_window_len.temp;
      output.ma_window_len.voltage = params.ma_window_len.voltage;
      output.ma_window_len.current = params.ma_window_len.current;
      output.roboteq.driver_state_timeout = params.roboteq.driver_state_timeout;
      output.battery_timeout = params.battery_timeout;

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
        if (param.get_name() == (prefix_ + "adc.device0")) {
            if(auto validation_result = not_empty<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.adc.device0 = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "adc.device1")) {
            if(auto validation_result = not_empty<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.adc.device1 = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "adc.ma_window_len.charge")) {
            if(auto validation_result = gt<int64_t>(param, 0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.adc.ma_window_len.charge = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "adc.ma_window_len.temp")) {
            if(auto validation_result = gt<int64_t>(param, 0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.adc.ma_window_len.temp = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "ma_window_len.voltage")) {
            if(auto validation_result = gt<int64_t>(param, 0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.ma_window_len.voltage = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "ma_window_len.current")) {
            if(auto validation_result = gt<int64_t>(param, 0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.ma_window_len.current = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "roboteq.driver_state_timeout")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.roboteq.driver_state_timeout = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "battery_timeout")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.battery_timeout = param.as_double();
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
      if (!parameters_interface_->has_parameter(prefix_ + "adc.device0")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Internal ADC0 IIO device.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.adc.device0);
          parameters_interface_->declare_parameter(prefix_ + "adc.device0", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "adc.device1")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Internal ADC1 IIO device.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.adc.device1);
          parameters_interface_->declare_parameter(prefix_ + "adc.device1", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "adc.ma_window_len.charge")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Window length of a moving average, used to smooth out battery charge readings.";
          descriptor.read_only = false;
          descriptor.integer_range.resize(1);
          descriptor.integer_range.at(0).from_value = 0;
          descriptor.integer_range.at(0).to_value = std::numeric_limits<int64_t>::max();
          auto parameter = to_parameter_value(updated_params.adc.ma_window_len.charge);
          parameters_interface_->declare_parameter(prefix_ + "adc.ma_window_len.charge", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "adc.ma_window_len.temp")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Window length of a moving average, used to smooth out battery temperature readings.";
          descriptor.read_only = false;
          descriptor.integer_range.resize(1);
          descriptor.integer_range.at(0).from_value = 0;
          descriptor.integer_range.at(0).to_value = std::numeric_limits<int64_t>::max();
          auto parameter = to_parameter_value(updated_params.adc.ma_window_len.temp);
          parameters_interface_->declare_parameter(prefix_ + "adc.ma_window_len.temp", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "ma_window_len.voltage")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Window length of a moving average, used to smooth out battery voltage readings.";
          descriptor.read_only = false;
          descriptor.integer_range.resize(1);
          descriptor.integer_range.at(0).from_value = 0;
          descriptor.integer_range.at(0).to_value = std::numeric_limits<int64_t>::max();
          auto parameter = to_parameter_value(updated_params.ma_window_len.voltage);
          parameters_interface_->declare_parameter(prefix_ + "ma_window_len.voltage", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "ma_window_len.current")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Window length of a moving average, used to smooth out battery current readings.";
          descriptor.read_only = false;
          descriptor.integer_range.resize(1);
          descriptor.integer_range.at(0).from_value = 0;
          descriptor.integer_range.at(0).to_value = std::numeric_limits<int64_t>::max();
          auto parameter = to_parameter_value(updated_params.ma_window_len.current);
          parameters_interface_->declare_parameter(prefix_ + "ma_window_len.current", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "roboteq.driver_state_timeout")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Timeout in seconds after which driver state messages will be considered old. Used as a fallback when ADC data is not available.";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.roboteq.driver_state_timeout);
          parameters_interface_->declare_parameter(prefix_ + "roboteq.driver_state_timeout", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "battery_timeout")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Timeout in seconds. If the node fails to read battery data exceeding this duration, the node will publish an unknown battery state.";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.battery_timeout);
          parameters_interface_->declare_parameter(prefix_ + "battery_timeout", parameter, descriptor);
      }
      // get parameters and fill struct fields
      rclcpp::Parameter param;
      param = parameters_interface_->get_parameter(prefix_ + "adc.device0");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = not_empty<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'adc.device0': {}", validation_result.error()));
      }
      updated_params.adc.device0 = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "adc.device1");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = not_empty<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'adc.device1': {}", validation_result.error()));
      }
      updated_params.adc.device1 = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "adc.ma_window_len.charge");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<int64_t>(param, 0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'adc.ma_window_len.charge': {}", validation_result.error()));
      }
      updated_params.adc.ma_window_len.charge = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "adc.ma_window_len.temp");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<int64_t>(param, 0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'adc.ma_window_len.temp': {}", validation_result.error()));
      }
      updated_params.adc.ma_window_len.temp = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "ma_window_len.voltage");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<int64_t>(param, 0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'ma_window_len.voltage': {}", validation_result.error()));
      }
      updated_params.ma_window_len.voltage = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "ma_window_len.current");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<int64_t>(param, 0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'ma_window_len.current': {}", validation_result.error()));
      }
      updated_params.ma_window_len.current = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "roboteq.driver_state_timeout");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'roboteq.driver_state_timeout': {}", validation_result.error()));
      }
      updated_params.roboteq.driver_state_timeout = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "battery_timeout");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'battery_timeout': {}", validation_result.error()));
      }
      updated_params.battery_timeout = param.as_double();


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
      rclcpp::Logger logger_ = rclcpp::get_logger("battery");
      std::mutex mutable mutex_;
  };

} // namespace battery
