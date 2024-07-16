/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2019 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

/*
* ######################################################################################
* # To use these examples:                                                             #
* #  - Connect to the robot's web page                                                 #
* #  - Select the Camera view page                                                     #
* #  - Observe the effects of changing the Color sensor options                        #
* ######################################################################################
*/

#include <SessionManager.h>
#include <DeviceManagerClientRpc.h>
#include <VisionConfigClientRpc.h>
#include <DeviceConfigClientRpc.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>

#include <thread>
#include <chrono>
#include <iomanip>      // for std::setprecision, std::setfill, std::setw

#include "utilities.h"

namespace k_api = Kinova::Api;

#define PORT 10000


typedef struct _option_info
{
    // int id;
    k_api::VisionConfig::Option id;
    string name;
    bool writable;
    float min_value;
    float max_value;
    float step_value;
    float default_value;
} option_info_t;

// Lists of supported options for a sensor
std::vector<option_info_t> supported_color_options;
std::vector<option_info_t> supported_depth_options;
// Empty list to use for an unsupported sensor
std::vector<option_info_t> no_options;
//
//
/**
 * Map of all Sensor strings
 */
std::map<k_api::VisionConfig::Sensor, std::string> map_all_sensor_strings =
{
    {k_api::VisionConfig::Sensor::SENSOR_UNSPECIFIED, "Unspecified sensor"},
    {k_api::VisionConfig::Sensor::SENSOR_COLOR, "Color"},
    {k_api::VisionConfig::Sensor::SENSOR_DEPTH, "Depth"},
};
//
/**
 * Map of all Resolution strings
 */
std::map<k_api::VisionConfig::Resolution, std::string> map_all_resolution_strings =
{
    {k_api::VisionConfig::Resolution::RESOLUTION_UNSPECIFIED, "Unspecified resolution"},
    {k_api::VisionConfig::Resolution::RESOLUTION_320x240, "320x240"},
    {k_api::VisionConfig::Resolution::RESOLUTION_424x240, "424x240"},
    {k_api::VisionConfig::Resolution::RESOLUTION_480x270, "480x270"},
    {k_api::VisionConfig::Resolution::RESOLUTION_640x480, "640x480"},
    {k_api::VisionConfig::Resolution::RESOLUTION_1280x720, "1280x720"},
    {k_api::VisionConfig::Resolution::RESOLUTION_1920x1080, "1920x1080"}
};
//
///*****************************
// * Example related functions *
// *****************************/
//
 /**
  * Returns a string matching the requested sensor
  */
std::string sensor_to_string(k_api::VisionConfig::Sensor sensor)
{
    std::string str;
    std::map<k_api::VisionConfig::Sensor, std::string>::iterator iter;

    iter = map_all_sensor_strings.find(sensor);
    if (iter != map_all_sensor_strings.end())
    {
        str = iter->second;
    }
    else
    {
        str = "Unknown sensor";
    }

    return str;
}
//
/**
 * Returns a string matching the requested resolution
 */
std::string resolution_to_string(k_api::VisionConfig::Resolution resolution)
{
    std::string str;
    std::map<k_api::VisionConfig::Resolution, std::string>::iterator iter;

    iter = map_all_resolution_strings.find(resolution);
    if (iter != map_all_resolution_strings.end())
    {
        str = iter->second;
    }
    else
    {
        str = "Unknown resolution";
    }

    return str;
}

/**
 * Prints the intrinsic parameters on stdout
 */
void print_intrinsic_parameters(const k_api::VisionConfig::IntrinsicParameters& intrinsics)
{
    // sets display precision for floating point values
    std::cout << std::fixed;
    std::cout << std::setprecision(6);

    std::cout << "Sensor: " << intrinsics.sensor();
    std::cout << " (" << sensor_to_string(intrinsics.sensor()) << ")" << std::endl;
    std::cout << "Resolution: " << intrinsics.resolution();
    std::cout << " (" << resolution_to_string(intrinsics.resolution()) << ")" << std::endl;
    std::cout << "Principal point x: " << intrinsics.principal_point_x() << std::endl;
    std::cout << "Principal point y: " << intrinsics.principal_point_y() << std::endl;
    std::cout << "Focal length x: " << intrinsics.focal_length_x() << std::endl;
    std::cout << "Focal length y: " << intrinsics.focal_length_y() << std::endl;
    std::cout << "Distortion coefficients: [" << intrinsics.distortion_coeffs().k1() << " " \
        << intrinsics.distortion_coeffs().k2() << " " \
        << intrinsics.distortion_coeffs().p1() << " " \
        << intrinsics.distortion_coeffs().p2() << " " \
        << intrinsics.distortion_coeffs().k3() << "]" \
        << std::endl;
}



/**
 * Returns the device identifier of the Vision module, 0 if not found
 */
uint32_t example_vision_get_device_id(k_api::DeviceManager::DeviceManagerClient* device_manager)
{
    uint32_t vision_device_id = 0;

    // Gets all device routing information (from DeviceManagerClient service)
    auto all_devices_info = device_manager->ReadAllDevices();

    // Uses device routing information to route to every device (base, actuator, interconnect, etc.)
    // in the armbase system and request general device information
    for (auto dev : all_devices_info.device_handle())
    {
        if (dev.device_type() == k_api::Common::DeviceTypes::VISION)
        {
            vision_device_id = dev.device_identifier();
            std::cout << "Vision module found, device Id: " << vision_device_id << std::endl;
            break;
        }
    }

    return vision_device_id;
}

///**
// * Display how to use these examples
// */
//void example_display_usage()
//{
//    std::cout << std::endl;
//    std::cout << "\t######################################################################################" << std::endl;
//    std::cout << "\t# To use these examples:                                                             #" << std::endl;
//    std::cout << "\t#  - Connect to the robot's web page                                                 #" << std::endl;
//    std::cout << "\t#  - Select the Camera view page                                                     #" << std::endl;
//    std::cout << "\t#  - Observe the effects of changing the Color sensor options                        #" << std::endl;
//    std::cout << "\t######################################################################################" << std::endl;
//    std::cout << std::endl;
//}
//
///**
// * Get the name of a sensor
// */
//string example_get_sensor_name(k_api::VisionConfig::Sensor sensor)
//{
//    string name;
//    if (sensor == k_api::VisionConfig::Sensor::SENSOR_COLOR)
//    {
//        name = "COLOR";
//    }
//    else if (sensor == k_api::VisionConfig::Sensor::SENSOR_DEPTH)
//    {
//        name = "DEPTH";
//    }
//    else
//    {
//        name = "***UNSUPPORTED***";
//    }
//
//    return name;
//}
//
///**
// * Get the list of supported options for a sensor
// * Each list item is a dictionary describing an option information
// */
//const std::vector<option_info_t>& example_get_sensor_supported_options(k_api::VisionConfig::Sensor sensor)
//{
//    if (sensor == k_api::VisionConfig::Sensor::SENSOR_COLOR)
//    {
//        return supported_color_options;
//    }
//    else if (sensor == k_api::VisionConfig::Sensor::SENSOR_DEPTH)
//    {
//        return supported_depth_options;
//    }
//    else
//    {
//        return no_options;
//    }
//}
//
///**
// * Display the information of a specific sensor option
// */
//void example_display_sensor_option_information(const option_info_t& option_info)
//{
//    // sets display precision for floating point values
//    std::cout << std::fixed;
//    std::cout << std::setprecision(6);
//
//    std::cout << "Option id: " << std::setfill('0') << std::setw(2) << option_info.id << "  name: " << option_info.name;
//    std::cout << "  is_writable: " << std::boolalpha << option_info.writable << std::endl;
//    std::cout << "minimum: " << option_info.min_value << "  maximum: " << option_info.max_value << std::endl;
//    std::cout << "   step: " << option_info.step_value << "  default: " << option_info.default_value << std::endl << std::endl;
//}
//
///**
// * Add option information to a list of supported options for a sensor
// * The added item is a dictionary describing the option information, with the following fields:
// *   'id', 'name', 'writable', 'min', 'max', 'step', 'default'
// * Then, display the option information
// */
//void example_add_and_display_sensor_supported_option(const k_api::VisionConfig::OptionInformation& option_info)
//{
//    bool display_option_info = true;
//    option_info_t option_infoData = {};
//
//    option_infoData.id = option_info.option();
//    option_infoData.name = k_api::VisionConfig::Option_Name(option_info.option());
//    option_infoData.writable = !option_info.read_only();
//    option_infoData.min_value = option_info.minimum();
//    option_infoData.max_value = option_info.maximum();
//    option_infoData.step_value = option_info.step();
//    option_infoData.default_value = option_info.default_value();
//
//    if (option_info.sensor() == k_api::VisionConfig::Sensor::SENSOR_COLOR)
//    {
//        supported_color_options.push_back(option_infoData);
//    }
//    else if (option_info.sensor() == k_api::VisionConfig::Sensor::SENSOR_DEPTH)
//    {
//        supported_depth_options.push_back(option_infoData);
//    }
//    else
//    {
//        std::cout << "Unsupported sensor " << option_info.sensor() << " for option id " << option_info.option();
//        std::cout << ", not adding to any list!" << std::endl;
//        display_option_info = false;
//    }
//
//    // Display option information
//    if (display_option_info)
//    {
//        example_display_sensor_option_information(option_infoData);
//    }
//}
//
///**
// * From the given option information, return the value requested by the range
// */
//float example_get_option_value_by_range(const option_info_t& option_info, const string& value_range)
//{
//    float value;
//
//    if (value_range == "min")
//    {
//        value = option_info.min_value;
//    }
//    else if (value_range == "max")
//    {
//        value = option_info.max_value;
//    }
//    else if (value_range == "default")
//    {
//        value = option_info.default_value;
//    }
//    else
//    {
//        value = 0.0f;
//    }
//
//    return value;
//}
//
///**
// * For all sensor options, set their value based on the specified range
// */
//void example_set_sensor_options_values_by_range(k_api::VisionConfig::Sensor sensor, const string& value_range,
//    k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id)
//{
//    k_api::VisionConfig::OptionValue option_value;
//    const std::vector<option_info_t>& sensor_options = example_get_sensor_supported_options(sensor);
//    std::vector<option_info_t>::const_iterator iter_options;
//
//    option_value.set_sensor(sensor);
//
//    for (iter_options = sensor_options.begin(); iter_options != sensor_options.end(); ++iter_options)
//    {
//        if (iter_options->writable)
//        {
//            try
//            {
//                option_value.set_option(iter_options->id);
//                option_value.set_value(example_get_option_value_by_range(*iter_options, value_range));
//                vision->SetOptionValue(option_value, device_id);
//                std::cout << "Set value (" << option_value.value() << ") for option '" << iter_options->name << "'" << std::endl;
//            }
//            catch (const std::exception& ex)
//            {
//                std::cout << "Failed to set " << value_range << " value for option '" << iter_options->name << "': " << ex.what() << std::endl;
//            }
//        }
//    }
//}
//
///**
// * For all sensor options, validate their value based on the specified range
// */
//void example_validate_sensor_options_values_by_range(k_api::VisionConfig::Sensor sensor, const string& value_range,
//    k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id)
//{
//    k_api::VisionConfig::OptionIdentifier option_identifier;
//    k_api::VisionConfig::OptionValue option_value_reply;
//    const std::vector<option_info_t>& sensor_options = example_get_sensor_supported_options(sensor);
//    std::vector<option_info_t>::const_iterator iter_options;
//
//    option_identifier.set_sensor(sensor);
//
//    for (iter_options = sensor_options.begin(); iter_options != sensor_options.end(); ++iter_options)
//    {
//        if (iter_options->writable)
//        {
//            try
//            {
//                option_identifier.set_option(iter_options->id);
//                option_value_reply = vision->GetOptionValue(option_identifier, device_id);
//                std::cout << "Confirm received value (" << option_value_reply.value() << ") for option '" << iter_options->name << "' --> ";
//                std::cout << (option_value_reply.value() == example_get_option_value_by_range(*iter_options, value_range) ? "OK" : "*** FAILED ***");
//                std::cout << std::endl;
//            }
//            catch (const std::exception& ex)
//            {
//                std::cout << "Failed to get value for option '" << iter_options->name << "': " << ex.what() << std::endl;
//            }
//        }
//    }
//}
//
///**
// * Example showing how to get the sensors options information
// * Note: This function must be called in order to fill up the lists of sensors supported options
// */
//void example_routed_vision_get_option_information(k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id)
//{
//    int sensor_index;
//    int option_index;
//    k_api::VisionConfig::OptionIdentifier option_identifier;
//    k_api::VisionConfig::OptionInformation option_info;
//    string sensor_name;
//
//    std::cout << "\n** Example showing how to get the sensors options information **" << std::endl;
//
//    // For all sensors, determine which options are supported and populate specific list
//    for (sensor_index = k_api::VisionConfig::Sensor_MIN + 1; sensor_index <= k_api::VisionConfig::Sensor_MAX; sensor_index++)
//    {
//        option_identifier.set_sensor(static_cast<k_api::VisionConfig::Sensor> (sensor_index));
//        sensor_name = example_get_sensor_name(static_cast<k_api::VisionConfig::Sensor> (sensor_index));
//        std::cout << "\n-- Using Vision Config Service to get information for all " << sensor_name << " sensor options --" << std::endl;
//        for (option_index = k_api::VisionConfig::Option_MIN + 1; option_index <= k_api::VisionConfig::Option_MAX; option_index++)
//        {
//            try
//            {
//                option_identifier.set_option(static_cast<k_api::VisionConfig::Option> (option_index));
//                option_info = vision->GetOptionInformation(option_identifier, device_id);
//                if (option_info.sensor() == static_cast<k_api::VisionConfig::Sensor> (sensor_index) &&
//                    option_info.option() == static_cast<k_api::VisionConfig::Option> (option_index))
//                {
//                    if (option_info.supported())
//                    {
//                        example_add_and_display_sensor_supported_option(option_info);
//                    }
//                }
//                else
//                {
//                    std::cout << "Unexpected mismatch of sensor or option in returned information for option id ";
//                    std::cout << option_index << "!" << std::endl;
//                }
//            }
//            catch (const std::exception& ex)
//            {
//                // The option is simply not supported
//            }
//        }
//    }
//}
//
///**
// * Example showing how to get the sensors options values
// */
//void example_routed_vision_get_sensor_options_values(k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id)
//{
//    int sensor_index;
//    k_api::VisionConfig::OptionIdentifier option_identifier;
//    k_api::VisionConfig::OptionValue option_value;
//    string sensor_name;
//    std::vector<option_info_t>::const_iterator iter_options;
//
//    std::cout << "\n** Example showing how to get the sensors options values **" << std::endl;
//
//    // sets display precision for floating point values
//    std::cout << std::fixed;
//    std::cout << std::setprecision(6);
//
//    // For all sensors, get their supported options value
//    for (sensor_index = k_api::VisionConfig::Sensor_MIN + 1; sensor_index <= k_api::VisionConfig::Sensor_MAX; sensor_index++)
//    {
//        option_identifier.set_sensor(static_cast<k_api::VisionConfig::Sensor> (sensor_index));
//        sensor_name = example_get_sensor_name(static_cast<k_api::VisionConfig::Sensor> (sensor_index));
//        const std::vector<option_info_t>& sensor_options = example_get_sensor_supported_options(static_cast<k_api::VisionConfig::Sensor> (sensor_index));
//        std::cout << "\n-- Using Vision Config Service to get value for all " << sensor_name << " sensor options --" << std::endl;
//        for (iter_options = sensor_options.begin(); iter_options != sensor_options.end(); ++iter_options)
//        {
//            try
//            {
//                option_identifier.set_option(iter_options->id);
//                option_value = vision->GetOptionValue(option_identifier, device_id);
//                std::cout << "Option '" << iter_options->name << "' has value " << option_value.value() << std::endl;
//            }
//            catch (const std::exception& ex)
//            {
//                std::cout << "Failed to get value of option '" << iter_options->name << "': " << ex.what() << std::endl;
//            }
//        }
//        std::cout << std::endl;
//    }
//}
//
///**
// * Example showing how to set the sensors options values
// */
//void example_routed_vision_set_sensor_options_values(k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id)
//{
//    int sensor_index;
//    string sensor_name;
//    std::vector<string> value_ranges = { "max", "default" };
//    std::vector<string>::iterator iter_value_ranges;
//
//    std::cout << "\n** Example showing how to set the sensors options values **" << std::endl;
//
//    // Set display precision for floating point values
//    std::cout << std::fixed;
//    std::cout << std::setprecision(6);
//
//    // For all sensors, set and confirm options values
//    for (iter_value_ranges = value_ranges.begin(); iter_value_ranges != value_ranges.end(); ++iter_value_ranges)
//    {
//        for (sensor_index = k_api::VisionConfig::Sensor_MIN + 1; sensor_index <= k_api::VisionConfig::Sensor_MAX; sensor_index++)
//        {
//            sensor_name = example_get_sensor_name(static_cast<k_api::VisionConfig::Sensor> (sensor_index));
//
//            std::cout << "\n-- Using Vision Config Service to set " << *iter_value_ranges << " value for all " << sensor_name;
//            std::cout << " sensor options --" << std::endl;
//            example_set_sensor_options_values_by_range(static_cast<k_api::VisionConfig::Sensor> (sensor_index), *iter_value_ranges, vision, device_id);
//
//            std::cout << "\n-- Using Vision Config Service to confirm " << *iter_value_ranges << " value was set for all " << sensor_name;
//            std::cout << " sensor options --" << std::endl;
//            example_validate_sensor_options_values_by_range(static_cast<k_api::VisionConfig::Sensor> (sensor_index), *iter_value_ranges, vision, device_id);
//
//            if (static_cast<k_api::VisionConfig::Sensor> (sensor_index) == k_api::VisionConfig::Sensor::SENSOR_COLOR)
//            {
//                std::cout << "\n-- Waiting for 5 seconds to observe the effects of the new COLOR sensor options values... --" << std::endl;
//                std::this_thread::sleep_for(std::chrono::seconds(5));
//            }
//            else
//            {
//                std::cout << std::endl;
//            }
//        }
//    }
//}
//
///**
// * Example confirming that sensors options values are restored upon a reboot of the Vision module
// */
//void example_routed_vision_confirm_saved_sensor_options_values(k_api::VisionConfig::VisionConfigClient* vision, k_api::DeviceConfig::DeviceConfigClient* device_config, uint32_t device_id)
//{
//    int sensor_index;
//    string sensor_name;
//    std::vector<string> value_ranges = { "min", "default" };
//    std::vector<string>::iterator iter_value_ranges;
//
//    std::cout << "\n** Example confirming that sensors options values are restored upon a reboot of the Vision module **" << std::endl;
//
//    // Set display precision for floating point values
//    std::cout << std::fixed;
//    std::cout << std::setprecision(6);
//
//    // For all sensors, set and confirm options values
//    for (iter_value_ranges = value_ranges.begin(); iter_value_ranges != value_ranges.end(); ++iter_value_ranges)
//    {
//        for (sensor_index = k_api::VisionConfig::Sensor_MIN + 1; sensor_index <= k_api::VisionConfig::Sensor_MAX; sensor_index++)
//        {
//            sensor_name = example_get_sensor_name(static_cast<k_api::VisionConfig::Sensor> (sensor_index));
//
//            std::cout << "\n-- Using Vision Config Service to set " << *iter_value_ranges << " value for all " << sensor_name;
//            std::cout << " sensor options --" << std::endl;
//            example_set_sensor_options_values_by_range(static_cast<k_api::VisionConfig::Sensor> (sensor_index), *iter_value_ranges, vision, device_id);
//
//            std::cout << "\n-- Using Vision Config Service to confirm " << *iter_value_ranges << " value was set for all " << sensor_name;
//            std::cout << " sensor options --" << std::endl;
//            example_validate_sensor_options_values_by_range(static_cast<k_api::VisionConfig::Sensor> (sensor_index), *iter_value_ranges, vision, device_id);
//        }
//
//        // If we just set the options' minimum value, reboot the Vision module device
//        if (*iter_value_ranges == "min")
//        {
//            // Reboot with a delay
//            int delayToReboot_ms = 5000;
//            k_api::DeviceConfig::RebootRqst rebootRequest;
//            rebootRequest.set_delay(delayToReboot_ms);
//            std::cout << "\n-- Using Device Config Service to reboot the Vision module in " << delayToReboot_ms << " milliseconds. ";
//            std::cout << "Please wait... --" << std::endl;
//            device_config->RebootRequest(rebootRequest, device_id);
//
//            // Wait until the Vision module is rebooted completely
//            int waitAfterRebootSec = 35 + (delayToReboot_ms / 1000);
//            std::this_thread::sleep_for(std::chrono::seconds(waitAfterRebootSec));
//
//            // For all sensors, confirm their option values were restored
//            for (sensor_index = k_api::VisionConfig::Sensor_MIN + 1; sensor_index <= k_api::VisionConfig::Sensor_MAX; sensor_index++)
//            {
//                sensor_name = example_get_sensor_name(static_cast<k_api::VisionConfig::Sensor> (sensor_index));
//
//                std::cout << "\n-- Using Vision Config Service to confirm " << *iter_value_ranges << " value was restored after reboot for all ";
//                std::cout << sensor_name << " sensor options --" << std::endl;
//                example_validate_sensor_options_values_by_range(static_cast<k_api::VisionConfig::Sensor> (sensor_index), *iter_value_ranges, vision, device_id);
//            }
//        }
//        else
//        {
//            std::cout << std::endl;
//        }
//    }
//}
//
/**
 * Example showing how to set the intrinsic parameters of the Color and Depth sensors
 */
void example_routed_vision_set_intrinsics(k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id)
{
    k_api::VisionConfig::IntrinsicParameters intrinsics_old;
    k_api::VisionConfig::IntrinsicParameters intrinsics_new;
    k_api::VisionConfig::IntrinsicParameters intrinsics_reply;
    k_api::VisionConfig::IntrinsicProfileIdentifier profile_id;

    std::cout << "\n\n** Example showing how to set the intrinsic parameters of the Color and Depth sensors **" << std::endl;

    std::cout << "\n-- Using Vision Config Service to get current intrinsic parameters for color resolution 640x480 --" << std::endl;
    profile_id.set_sensor(k_api::VisionConfig::SENSOR_COLOR);
    profile_id.set_resolution(k_api::VisionConfig::RESOLUTION_640x480);
    intrinsics_old = vision->GetIntrinsicParametersProfile(profile_id, device_id);
    print_intrinsic_parameters(intrinsics_old);

    std::cout << "\n-- Using Vision Config Service to set new intrinsic parameters for color resolution 640x480 --" << std::endl;
    intrinsics_new.set_sensor(profile_id.sensor());
    intrinsics_new.set_resolution(profile_id.resolution());
    intrinsics_new.set_principal_point_x(640 / 2 + 0.123456f);
    intrinsics_new.set_principal_point_y(480 / 2 + 1.789012f);
    intrinsics_new.set_focal_length_x(650.567890f);
    intrinsics_new.set_focal_length_y(651.112233f);
    intrinsics_new.mutable_distortion_coeffs()->set_k1(0.2f);
    intrinsics_new.mutable_distortion_coeffs()->set_k2(0.05f);
    intrinsics_new.mutable_distortion_coeffs()->set_p1(1.2f);
    intrinsics_new.mutable_distortion_coeffs()->set_p2(0.999999f);
    intrinsics_new.mutable_distortion_coeffs()->set_k3(0.001f);
    vision->SetIntrinsicParameters(intrinsics_new, device_id);

    std::cout << "\n-- Using Vision Config Service to get new intrinsic parameters for color resolution 640x480 --" << std::endl;
    intrinsics_reply = vision->GetIntrinsicParametersProfile(profile_id, device_id);
    print_intrinsic_parameters(intrinsics_reply);

    std::cout << "\n-- Using Vision Config Service to set back old intrinsic parameters for color resolution 640x480 --" << std::endl;
    vision->SetIntrinsicParameters(intrinsics_old, device_id);


    std::cout << "\n-- Using Vision Config Service to get current intrinsic parameters for depth resolution 424x240 --" << std::endl;
    profile_id.set_sensor(k_api::VisionConfig::SENSOR_DEPTH);
    profile_id.set_resolution(k_api::VisionConfig::RESOLUTION_424x240);
    intrinsics_old = vision->GetIntrinsicParametersProfile(profile_id, device_id);
    print_intrinsic_parameters(intrinsics_old);

    std::cout << "\n-- Using Vision Config Service to set new intrinsic parameters for depth resolution 424x240 --" << std::endl;
    intrinsics_new.set_sensor(profile_id.sensor());
    intrinsics_new.set_resolution(profile_id.resolution());
    intrinsics_new.set_principal_point_x(424 / 2 + 0.123456f);
    intrinsics_new.set_principal_point_y(240 / 2 + 1.789012f);
    intrinsics_new.set_focal_length_x(315.567890f);
    intrinsics_new.set_focal_length_y(317.112233f);
    intrinsics_new.mutable_distortion_coeffs()->set_k1(0.425f);
    intrinsics_new.mutable_distortion_coeffs()->set_k2(1.735102f);
    intrinsics_new.mutable_distortion_coeffs()->set_p1(0.1452f);
    intrinsics_new.mutable_distortion_coeffs()->set_p2(0.767574f);
    intrinsics_new.mutable_distortion_coeffs()->set_k3(2.345678f);
    vision->SetIntrinsicParameters(intrinsics_new, device_id);

    std::cout << "\n-- Using Vision Config Service to get new intrinsic parameters for depth resolution 424x240 --" << std::endl;
    intrinsics_reply = vision->GetIntrinsicParametersProfile(profile_id, device_id);
    print_intrinsic_parameters(intrinsics_reply);

    std::cout << "\n-- Using Vision Config Service to set back old intrinsic parameters for depth resolution 424x240 --" << std::endl;
    vision->SetIntrinsicParameters(intrinsics_old, device_id);
}

/**
 * Example showing how to retrieve the intrinsic parameters of the Color and Depth sensors
 */
void example_routed_vision_get_intrinsics(k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id)
{
    k_api::VisionConfig::IntrinsicParameters intrinsics;
    k_api::VisionConfig::SensorIdentifier sensor_id;
    k_api::VisionConfig::IntrinsicProfileIdentifier profile_id;

    std::cout << "\n\n** Example showing how to retrieve the intrinsic parameters of the Color and Depth sensors **" << std::endl;

    std::cout << "\n-- Using Vision Config Service to get intrinsic parameters of active color resolution --" << std::endl;
    sensor_id.set_sensor(k_api::VisionConfig::SENSOR_COLOR);

    intrinsics = vision->GetIntrinsicParameters(sensor_id, device_id);
    print_intrinsic_parameters(intrinsics);

    std::cout << "\n-- Using Vision Config Service to get intrinsic parameters of active depth resolution --" << std::endl;
    sensor_id.set_sensor(k_api::VisionConfig::SENSOR_DEPTH);

    intrinsics = vision->GetIntrinsicParameters(sensor_id, device_id);
    print_intrinsic_parameters(intrinsics);

    std::cout << "\n-- Using Vision Config Service to get intrinsic parameters for color resolution 1920x1080 --" << std::endl;
    profile_id.set_sensor(k_api::VisionConfig::SENSOR_COLOR);
    profile_id.set_resolution(k_api::VisionConfig::RESOLUTION_1920x1080);

    intrinsics = vision->GetIntrinsicParametersProfile(profile_id, device_id);
    print_intrinsic_parameters(intrinsics);

    std::cout << "\n-- Using Vision Config Service to get intrinsic parameters for depth resolution 424x240 --" << std::endl;
    profile_id.set_sensor(k_api::VisionConfig::SENSOR_DEPTH);
    profile_id.set_resolution(k_api::VisionConfig::RESOLUTION_424x240);

    intrinsics = vision->GetIntrinsicParametersProfile(profile_id, device_id);
    print_intrinsic_parameters(intrinsics);
}


//
///*****************************
// * Example related functions *
// *****************************/
//
// /**
//  * Returns a string matching the requested sensor
//  */
//std::string sensor_to_string(k_api::VisionConfig::Sensor sensor)
//{
//    std::string str;
//    std::map<k_api::VisionConfig::Sensor, std::string>::iterator iter;
//
//    iter = map_all_sensor_strings.find(sensor);
//    if (iter != map_all_sensor_strings.end())
//    {
//        str = iter->second;
//    }
//    else
//    {
//        str = "Unknown sensor";
//    }
//
//    return str;
//}
//
///**
// * Returns a string matching the requested resolution
// */
//std::string resolution_to_string(k_api::VisionConfig::Resolution resolution)
//{
//    std::string str;
//    std::map<k_api::VisionConfig::Resolution, std::string>::iterator iter;
//
//    iter = map_all_resolution_strings.find(resolution);
//    if (iter != map_all_resolution_strings.end())
//    {
//        str = iter->second;
//    }
//    else
//    {
//        str = "Unknown resolution";
//    }
//
//    return str;
//}
//
///**
// * Prints the intrinsic parameters on stdout
// */
//void print_intrinsic_parameters(const k_api::VisionConfig::IntrinsicParameters& intrinsics)
//{
//    // sets display precision for floating point values
//    std::cout << std::fixed;
//    std::cout << std::setprecision(6);
//
//    std::cout << "Sensor: " << intrinsics.sensor();
//    std::cout << " (" << sensor_to_string(intrinsics.sensor()) << ")" << std::endl;
//    std::cout << "Resolution: " << intrinsics.resolution();
//    std::cout << " (" << resolution_to_string(intrinsics.resolution()) << ")" << std::endl;
//    std::cout << "Principal point x: " << intrinsics.principal_point_x() << std::endl;
//    std::cout << "Principal point y: " << intrinsics.principal_point_y() << std::endl;
//    std::cout << "Focal length x: " << intrinsics.focal_length_x() << std::endl;
//    std::cout << "Focal length y: " << intrinsics.focal_length_y() << std::endl;
//    std::cout << "Distortion coefficients: [" << intrinsics.distortion_coeffs().k1() << " " \
//        << intrinsics.distortion_coeffs().k2() << " " \
//        << intrinsics.distortion_coeffs().p1() << " " \
//        << intrinsics.distortion_coeffs().p2() << " " \
//        << intrinsics.distortion_coeffs().k3() << "]" \
//        << std::endl;
//}
//
 /**
  * Prints the extrinsic parameters on stdout
  */
void print_extrinsic_parameters(const k_api::VisionConfig::ExtrinsicParameters& extrinsics)
{
    // sets display precision for floating point values
    std::cout << std::fixed;
    std::cout << std::setprecision(6);

    std::cout << "Rotation matrix:\n[" << std::setw(9) << extrinsics.rotation().row1().column1() << " " \
        << std::setw(9) << extrinsics.rotation().row1().column2() << " " \
        << std::setw(9) << extrinsics.rotation().row1().column3() << "\n " \
        << std::setw(9) << extrinsics.rotation().row2().column1() << " " \
        << std::setw(9) << extrinsics.rotation().row2().column2() << " " \
        << std::setw(9) << extrinsics.rotation().row2().column3() << "\n " \
        << std::setw(9) << extrinsics.rotation().row3().column1() << " " \
        << std::setw(9) << extrinsics.rotation().row3().column2() << " " \
        << std::setw(9) << extrinsics.rotation().row3().column3() << "]" \
        << std::endl;
    std::cout << "Translation vector: [" << extrinsics.translation().t_x() << " " \
        << extrinsics.translation().t_y() << " " \
        << extrinsics.translation().t_z() << "]" \
        << std::endl;
}






void my_print_INTRINSICS_EXTRINSICS(k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id)
{
    k_api::VisionConfig::IntrinsicParameters intrinsics;
    k_api::VisionConfig::ExtrinsicParameters extrinsics;
    k_api::VisionConfig::SensorIdentifier sensor_id;
    k_api::VisionConfig::IntrinsicProfileIdentifier profile_id;

    std::cout << "\n-- Using Vision Config Service to get intrinsic parameters of active color resolution --" << std::endl;
    sensor_id.set_sensor(k_api::VisionConfig::SENSOR_COLOR);

    intrinsics = vision->GetIntrinsicParameters(sensor_id, device_id);
    print_intrinsic_parameters(intrinsics);

    std::cout << "\n-- Using Vision Config Service to get intrinsic parameters of active depth resolution --" << std::endl;
    sensor_id.set_sensor(k_api::VisionConfig::SENSOR_DEPTH);

    intrinsics = vision->GetIntrinsicParameters(sensor_id, device_id);
    print_intrinsic_parameters(intrinsics);

    
    std::cout << "\n-- Using Vision Config Service to get the extrinsic parameters --" << std::endl;
    extrinsics = vision->GetExtrinsicParameters(device_id);
    print_extrinsic_parameters(extrinsics);

}




void my_set_resolution_1920x1080_480x270(k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id)
{
    //k_api::VisionConfig::IntrinsicParameters intrinsics;
    //k_api::VisionConfig::SensorIdentifier sensor_id;
    //k_api::VisionConfig::IntrinsicProfileIdentifier profile_id;

    //std::cout << "\n-- Using Vision Config Service to get intrinsic parameters for color resolution 1920x1080 --" << std::endl;
    //profile_id.set_sensor(k_api::VisionConfig::SENSOR_COLOR);
    //profile_id.set_resolution(k_api::VisionConfig::RESOLUTION_1920x1080);
    //intrinsics = vision->GetIntrinsicParametersProfile(profile_id, device_id);
    //print_intrinsic_parameters(intrinsics);

    //std::cout << "\n-- Using Vision Config Service to get intrinsic parameters for depth resolution 480x270 --" << std::endl;
    //profile_id.set_sensor(k_api::VisionConfig::SENSOR_DEPTH);
    //profile_id.set_resolution(k_api::VisionConfig::RESOLUTION_480x270);
    //intrinsics = vision->GetIntrinsicParametersProfile(profile_id, device_id);
    //print_intrinsic_parameters(intrinsics);




    k_api::VisionConfig::IntrinsicParameters intrinsics_old;
    k_api::VisionConfig::IntrinsicParameters current_intrinsics;
    k_api::VisionConfig::IntrinsicParameters intrinsics_new;
    k_api::VisionConfig::IntrinsicParameters intrinsics_reply;
    k_api::VisionConfig::IntrinsicProfileIdentifier profile_id;

    //std::cout << "\n\n** Example showing how to set the intrinsic parameters of the Color and Depth sensors **" << std::endl;

    std::cout << "\n-- Using Vision Config Service to get current intrinsic parameters for color resolution 1920x1080 --" << std::endl;
    profile_id.set_sensor(k_api::VisionConfig::SENSOR_COLOR);
    profile_id.set_resolution(k_api::VisionConfig::RESOLUTION_320x240);
    intrinsics_old = vision->GetIntrinsicParametersProfile(profile_id, device_id);
    //print_intrinsic_parameters(intrinsics_old);

    std::cout << "\n-- Using Vision Config Service to set new intrinsic parameters for color resolution 1920x1080 --" << std::endl;
    intrinsics_new.set_sensor(profile_id.sensor());
    intrinsics_new.set_resolution(profile_id.resolution());
    intrinsics_new.set_principal_point_x(intrinsics_old.principal_point_x());
    intrinsics_new.set_principal_point_y(intrinsics_old.principal_point_y());
    intrinsics_new.set_focal_length_x(intrinsics_old.focal_length_x());
    intrinsics_new.set_focal_length_y(intrinsics_old.focal_length_y());
    intrinsics_new.mutable_distortion_coeffs()->set_k1(intrinsics_old.distortion_coeffs().k1());
    intrinsics_new.mutable_distortion_coeffs()->set_k2(intrinsics_old.distortion_coeffs().k2());
    intrinsics_new.mutable_distortion_coeffs()->set_p1(intrinsics_old.distortion_coeffs().p1());
    intrinsics_new.mutable_distortion_coeffs()->set_p2(intrinsics_old.distortion_coeffs().p2());
    intrinsics_new.mutable_distortion_coeffs()->set_k3(intrinsics_old.distortion_coeffs().k3());
    vision->SetIntrinsicParameters(intrinsics_new, device_id);
    
    std::cout << "######################################################################################### Intrinsics after vision module is set up to be 1920x1080" << std::endl;
    my_print_INTRINSICS_EXTRINSICS(vision, device_id);

    std::cout << "\n-- Using Vision Config Service to get current intrinsic parameters for depth resolution 480x270 --" << std::endl;
    profile_id.set_sensor(k_api::VisionConfig::SENSOR_DEPTH);
    profile_id.set_resolution(k_api::VisionConfig::RESOLUTION_480x270);
    intrinsics_old = vision->GetIntrinsicParametersProfile(profile_id, device_id);
    //print_intrinsic_parameters(intrinsics_old);

    std::cout << "\n-- Using Vision Config Service to set new intrinsic parameters for depth resolution 424x240 --" << std::endl;
    intrinsics_new.set_sensor(profile_id.sensor());
    intrinsics_new.set_resolution(profile_id.resolution());
    intrinsics_new.set_principal_point_x(intrinsics_old.principal_point_x());
    intrinsics_new.set_principal_point_y(intrinsics_old.principal_point_y());
    intrinsics_new.set_focal_length_x(intrinsics_old.focal_length_x());
    intrinsics_new.set_focal_length_y(intrinsics_old.focal_length_y());
    intrinsics_new.mutable_distortion_coeffs()->set_k1(intrinsics_old.distortion_coeffs().k1());
    intrinsics_new.mutable_distortion_coeffs()->set_k2(intrinsics_old.distortion_coeffs().k2());
    intrinsics_new.mutable_distortion_coeffs()->set_p1(intrinsics_old.distortion_coeffs().p1());
    intrinsics_new.mutable_distortion_coeffs()->set_p2(intrinsics_old.distortion_coeffs().p2());
    intrinsics_new.mutable_distortion_coeffs()->set_k3(intrinsics_old.distortion_coeffs().k3());
    vision->SetIntrinsicParameters(intrinsics_new, device_id);

    current_intrinsics = vision->GetIntrinsicParametersProfile(profile_id, device_id);
    print_intrinsic_parameters(intrinsics_old);
}

/**
 * Example showing how to retrieve the extrinsic parameters
 */
void example_routed_vision_get_extrinsics(k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id)
{
    k_api::VisionConfig::ExtrinsicParameters extrinsics;

    std::cout << "\n\n** Example showing how to retrieve the extrinsic parameters **" << std::endl;

    std::cout << "\n-- Using Vision Config Service to get the extrinsic parameters --" << std::endl;
    extrinsics = vision->GetExtrinsicParameters(device_id);
    print_extrinsic_parameters(extrinsics);
}

/**
 * Example showing how to set the extrinsic parameters
 */
void example_routed_vision_set_extrinsics(k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id)
{
    k_api::VisionConfig::ExtrinsicParameters extrinsics_old;
    k_api::VisionConfig::ExtrinsicParameters extrinsics_new;
    k_api::VisionConfig::ExtrinsicParameters extrinsics_reply;

    std::cout << "\n\n** Example showing how to set the extrinsic parameters **" << std::endl;

    std::cout << "\n-- Using Vision Config Service to get current extrinsic parameters --" << std::endl;
    extrinsics_old = vision->GetExtrinsicParameters(device_id);
    print_extrinsic_parameters(extrinsics_old);

    std::cout << "\n-- Using Vision Config Service to set new extrinsic parameters --" << std::endl;
    extrinsics_new.mutable_rotation()->mutable_row1()->set_column1(1.0001f);
    extrinsics_new.mutable_rotation()->mutable_row1()->set_column2(0.1f);
    extrinsics_new.mutable_rotation()->mutable_row1()->set_column3(-0.01f);
    extrinsics_new.mutable_rotation()->mutable_row2()->set_column1(-0.001f);
    extrinsics_new.mutable_rotation()->mutable_row2()->set_column2(1.0002f);
    extrinsics_new.mutable_rotation()->mutable_row2()->set_column3(0.0001f);
    extrinsics_new.mutable_rotation()->mutable_row3()->set_column1(0.00001f);
    extrinsics_new.mutable_rotation()->mutable_row3()->set_column2(-0.000001f);
    extrinsics_new.mutable_rotation()->mutable_row3()->set_column3(1.0003f);
    extrinsics_new.mutable_translation()->set_t_x(-0.026123456f);
    extrinsics_new.mutable_translation()->set_t_y(-0.009876543f);
    extrinsics_new.mutable_translation()->set_t_z(0.00002f);
    vision->SetExtrinsicParameters(extrinsics_new, device_id);

    std::cout << "\n-- Using Vision Config Service to get new extrinsic parameters --" << std::endl;
    extrinsics_reply = vision->GetExtrinsicParameters(device_id);
    print_extrinsic_parameters(extrinsics_reply);

    std::cout << "\n-- Using Vision Config Service to set back old extrinsic parameters --" << std::endl;
    vision->SetExtrinsicParameters(extrinsics_old, device_id);
}

/**
 * Wait for 10 seconds, allowing to see the effects of the focus action
 */
void example_wait_for_focus_action()
{
    std::cout << "-- Waiting for 10 seconds to observe the effects of the focus action... --" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));
}

/**
 * Example showing how to play with the auto-focus of the Color camera
 */
void example_routed_vision_do_autofocus_action(k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id)
{
    k_api::VisionConfig::SensorFocusAction sensor_focus_action;
    sensor_focus_action.set_sensor(k_api::VisionConfig::Sensor::SENSOR_COLOR);

    std::cout << "\n\n** Example showing how to play with the auto-focus of the Color camera **" << std::endl;

    std::cout << "\n-- Using Vision Config Service to disable the auto-focus --" << std::endl;
    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_DISABLE_FOCUS);
    vision->DoSensorFocusAction(sensor_focus_action, device_id);
    std::cout << "-- Place or remove an object from the center of the camera, observe the focus doesn't change --" << std::endl;
    example_wait_for_focus_action();

    std::cout << "\n-- Using Vision Config Service to enable the auto-focus --" << std::endl;
    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_START_CONTINUOUS_FOCUS);
    vision->DoSensorFocusAction(sensor_focus_action, device_id);
    std::cout << "-- Place an object in the center of the camera, observe the focus adjusts automatically --" << std::endl;
    example_wait_for_focus_action();

    std::cout << "\n-- Using Vision Config Service to pause the auto-focus --" << std::endl;
    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_PAUSE_CONTINUOUS_FOCUS);
    vision->DoSensorFocusAction(sensor_focus_action, device_id);
    std::cout << "-- Move the object away from the center of the camera and then back, but at a different distance, observe the focus doesn't change --" << std::endl;
    example_wait_for_focus_action();

    std::cout << "\n-- Using Vision Config Service to focus now --" << std::endl;
    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_FOCUS_NOW);
    vision->DoSensorFocusAction(sensor_focus_action, device_id);
    std::cout << "-- Observe the focus tried to adjust to the object in front to the camera --" << std::endl;
    example_wait_for_focus_action();

    std::cout << "\n-- Using Vision Config Service to re-enable the auto-focus --" << std::endl;
    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_START_CONTINUOUS_FOCUS);
    vision->DoSensorFocusAction(sensor_focus_action, device_id);
    std::cout << "-- Move the object away from the center of the camera and then back, but at a different distance, observe the focus adjusts automatically --" << std::endl;
    example_wait_for_focus_action();
}

///**
// * Example showing how to set the focus of the Color camera to a X-Y point in the camera image
// */
//void example_routed_vision_set_focus_point(k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id)
//{
//    k_api::VisionConfig::SensorFocusAction sensor_focus_action;
//    sensor_focus_action.set_sensor(k_api::VisionConfig::Sensor::SENSOR_COLOR);
//
//    std::cout << "\n\n** Example showing how to set the focus of the Color camera to a X-Y point in the camera image **" << std::endl;
//
//    std::cout << "\n-- Using Vision Config Service to set the focus point in the center of the lower right quadrant of the camera image --" << std::endl;
//    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_SET_FOCUS_POINT);
//    sensor_focus_action.mutable_focus_point()->set_x(1280 * 3 / 4);
//    sensor_focus_action.mutable_focus_point()->set_y(720 * 3 / 4);
//    vision->DoSensorFocusAction(sensor_focus_action, device_id);
//    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_FOCUS_NOW);
//    vision->DoSensorFocusAction(sensor_focus_action, device_id);
//    std::cout << "-- Place an object in the center of the lower right quadrant of the camera image, observe the object gets into focus --" << std::endl;
//    example_wait_for_focus_action();
//
//    std::cout << "\n-- Using Vision Config Service to set the focus point back in the middle the camera image--" << std::endl;
//    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_SET_FOCUS_POINT);
//    sensor_focus_action.mutable_focus_point()->set_x(1280 / 2);
//    sensor_focus_action.mutable_focus_point()->set_y(720 / 2);
//    vision->DoSensorFocusAction(sensor_focus_action, device_id);
//    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_FOCUS_NOW);
//    vision->DoSensorFocusAction(sensor_focus_action, device_id);
//    std::cout << "-- Place an object in the center of the camera image, observe the object gets into focus --" << std::endl;
//    example_wait_for_focus_action();
//}
//
/**
 * Example showing how to set the manual focus of the Color camera (changes the focus distance)
 */
void example_routed_vision_set_manual_focus(k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id)
{
    k_api::VisionConfig::SensorFocusAction sensor_focus_action;
    sensor_focus_action.set_sensor(k_api::VisionConfig::Sensor::SENSOR_COLOR);

    std::cout << "\n\n** Example showing how to set the manual focus of the Color camera (changes the focus distance) **" << std::endl;

    std::cout << "\n-- Using Vision Config Service to set the manual focus on a very close object (close-up view) --" << std::endl;
    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_SET_MANUAL_FOCUS);
    sensor_focus_action.mutable_manual_focus()->set_value(1023); // Maximum accepted value
    vision->DoSensorFocusAction(sensor_focus_action, device_id);
    std::cout << "-- Place an object at around 2 inches away from the center of the camera, observe the object is in focus --" << std::endl;
    example_wait_for_focus_action();

    std::cout << "\n-- Using Vision Config Service to set the manual focus on an object at a greater distance --" << std::endl;
    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_SET_MANUAL_FOCUS);
    sensor_focus_action.mutable_manual_focus()->set_value(0); // Mininum accepted value
    vision->DoSensorFocusAction(sensor_focus_action, device_id);
    std::cout << "-- Move the object away from the camera until it gets into focus --" << std::endl;
    example_wait_for_focus_action();

    std::cout << "\n-- Using Vision Config Service to set the manual focus on a relatively close object (normal view) --" << std::endl;
    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_SET_MANUAL_FOCUS);
    sensor_focus_action.mutable_manual_focus()->set_value(350);
    vision->DoSensorFocusAction(sensor_focus_action, device_id);
    std::cout << "-- Move the object at around 8 inches away from the center of the camera, observe the object is in focus --" << std::endl;
    example_wait_for_focus_action();

    std::cout << "\n-- Using Vision Config Service to re-enable the auto-focus --" << std::endl;
    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_START_CONTINUOUS_FOCUS);
    vision->DoSensorFocusAction(sensor_focus_action, device_id);
    std::cout << "-- Move the object away from the camera and then back, but at a different distance, observe the focus adjusts automatically --" << std::endl;
    example_wait_for_focus_action();
}



// Taking an integer input - begins

std::istringstream ask_user_input(const std::string& prompt)
{
    std::cout << prompt;
    std::string s;
    getline(std::cin, s);
    return std::istringstream(s);
}

int ask_integer(const std::string& prompt)
{
    auto input = ask_user_input(prompt);
    int n;
    if ((input >> n) && (input >> std::ws).eof())
        return n;
}

// Taking an integer input - ends

void my_set_focus_distance(k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id)
{
    int focus_dist = ask_integer("Choose focus distance - 1023 for very close object (2 inches away), 350 (8 inches away) and 0 to focus on a farther object\n");
    k_api::VisionConfig::SensorFocusAction sensor_focus_action;
    sensor_focus_action.set_sensor(k_api::VisionConfig::Sensor::SENSOR_COLOR);

    //std::cout << "\n\n** Example showing how to set the manual focus of the Color camera (changes the focus distance) **" << std::endl;

    //std::cout << "\n-- Using Vision Config Service to set the manual focus on a very close object (close-up view) --" << std::endl;
    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_SET_MANUAL_FOCUS);
    //sensor_focus_action.mutable_manual_focus()->set_value(1023); // Maximum accepted value
    sensor_focus_action.mutable_manual_focus()->set_value(focus_dist); // Maximum accepted value
    vision->DoSensorFocusAction(sensor_focus_action, device_id);
    //std::cout << "-- Place an object at around 2 inches away from the center of the camera, observe the object is in focus --" << std::endl;
    example_wait_for_focus_action();

    //std::cout << "\n-- Using Vision Config Service to set the manual focus on an object at a greater distance --" << std::endl;
    //sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_SET_MANUAL_FOCUS);
    //sensor_focus_action.mutable_manual_focus()->set_value(0); // Mininum accepted value
    //vision->DoSensorFocusAction(sensor_focus_action, device_id);
    //std::cout << "-- Move the object away from the camera until it gets into focus --" << std::endl;
    //example_wait_for_focus_action();

    //std::cout << "\n-- Using Vision Config Service to set the manual focus on a relatively close object (normal view) --" << std::endl;
    //sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_SET_MANUAL_FOCUS);
    //sensor_focus_action.mutable_manual_focus()->set_value(350);
    //vision->DoSensorFocusAction(sensor_focus_action, device_id);
    //std::cout << "-- Move the object at around 8 inches away from the center of the camera, observe the object is in focus --" << std::endl;
    //example_wait_for_focus_action();
}

void my_autofocus_ON(k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id) {

    k_api::VisionConfig::SensorFocusAction sensor_focus_action;
    sensor_focus_action.set_sensor(k_api::VisionConfig::Sensor::SENSOR_COLOR);
    std::cout << "\n-- Using Vision Config Service to re-enable the auto-focus --" << std::endl;
    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_START_CONTINUOUS_FOCUS);
    vision->DoSensorFocusAction(sensor_focus_action, device_id);
    std::cout << "-- Move the object away from the camera and then back, but at a different distance, observe the focus adjusts automatically --" << std::endl;
    example_wait_for_focus_action();
}

/**
 * Display how to use these examples
 */
void example_display_usage()
{
    std::cout << std::endl;
    std::cout << "\t######################################################################################" << std::endl;
    std::cout << "\t# To use these examples:                                                             #" << std::endl;
    std::cout << "\t#  - Connect to the robot's web page                                                 #" << std::endl;
    std::cout << "\t#  - Configure the Vision Color Sensor to 1280x720 resolution                        #" << std::endl;
    std::cout << "\t#  - Position the robot so you can easily place objects in front of the Color camera #" << std::endl;
    std::cout << "\t#  - Select the Camera view page                                                     #" << std::endl;
    std::cout << "\t######################################################################################" << std::endl;
    std::cout << std::endl;
}


/// <summary>
/// //////////////////////////////////////
/// </summary>
std::string ROBOT_IP;

//my custom functions
ExampleArgs myParseExampleArguments(int argc, char* argv[])
{
    printf("Enter IP Address of the robot\n");
    char myIP_input[20];
    scanf("%[^\n]%*c", myIP_input);
    printf("IP Address entered is : %s\n", myIP_input);

    std::string ROBOT_IP2(myIP_input);
    ROBOT_IP = ROBOT_IP2;

    cxxopts::Options options(argv[0], "Kortex example");

    options.add_options()
        ("ip", "IP address of destination", cxxopts::value<std::string>()->default_value(ROBOT_IP))
        ("h,help", "Print help")
        ("u,username", "username to login", cxxopts::value<std::string>()->default_value("admin"))
        ("p,password", "password to login", cxxopts::value<std::string>()->default_value("admin"))
        ;

    ExampleArgs resultArgs;

    try
    {
        auto parsed_options = options.parse(argc, argv);

        if (parsed_options.count("help"))
        {
            std::cout << options.help() << std::endl;
            exit(EXIT_SUCCESS);
        }

        resultArgs.ip_address = parsed_options["ip"].as<std::string>();
        resultArgs.username = parsed_options["username"].as<std::string>();
        resultArgs.password = parsed_options["password"].as<std::string>();
    }
    catch (const cxxopts::OptionException& exception)
    {
        std::cerr << exception.what() << std::endl;
        exit(EXIT_FAILURE);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        exit(EXIT_FAILURE);
    }

    return resultArgs;
}


void my_disable_focus(k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id)
{
    k_api::VisionConfig::SensorFocusAction sensor_focus_action;
    sensor_focus_action.set_sensor(k_api::VisionConfig::Sensor::SENSOR_COLOR);

    std::cout << "\n\n** Example showing how to play with the auto-focus of the Color camera **" << std::endl;

    std::cout << "\n-- Using Vision Config Service to disable the auto-focus --" << std::endl;
    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_DISABLE_FOCUS);
    vision->DoSensorFocusAction(sensor_focus_action, device_id);
    std::cout << "-- Place or remove an object from the center of the camera, observe the focus doesn't change --" << std::endl;
}


void my_set_focus(k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id, float x_fraction, float y_fraction)
{
    k_api::VisionConfig::SensorFocusAction sensor_focus_action;
    sensor_focus_action.set_sensor(k_api::VisionConfig::Sensor::SENSOR_COLOR);

    std::cout << "\n\n** Example showing how to set the focus of the Color camera to a X-Y point in the camera image **" << std::endl;

    std::cout << "\n-- Using Vision Config Service to set the focus point in the center of the lower right quadrant of the camera image --" << std::endl;
    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_SET_FOCUS_POINT);
    sensor_focus_action.mutable_focus_point()->set_x(1280 * x_fraction);
    sensor_focus_action.mutable_focus_point()->set_y(720 * y_fraction);
    vision->DoSensorFocusAction(sensor_focus_action, device_id);
    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_FOCUS_NOW);
    vision->DoSensorFocusAction(sensor_focus_action, device_id);
    std::cout << "-- Place an object in the center of the lower right quadrant of the camera image, observe the object gets into focus --" << std::endl;
    example_wait_for_focus_action();
}

void my_set_focus_1920x1080(k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id, float x_fraction, float y_fraction)
{
    k_api::VisionConfig::SensorFocusAction sensor_focus_action;
    sensor_focus_action.set_sensor(k_api::VisionConfig::Sensor::SENSOR_COLOR);

    std::cout << "\n\n** Example showing how to set the focus of the Color camera to a X-Y point in the camera image **" << std::endl;

    std::cout << "\n-- Using Vision Config Service to set the focus point in the center of the lower right quadrant of the camera image --" << std::endl;
    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_SET_FOCUS_POINT);
    sensor_focus_action.mutable_focus_point()->set_x(1920 * x_fraction);
    sensor_focus_action.mutable_focus_point()->set_y(1080 * y_fraction);
    vision->DoSensorFocusAction(sensor_focus_action, device_id);
    sensor_focus_action.set_focus_action(k_api::VisionConfig::FocusAction::FOCUSACTION_FOCUS_NOW);
    vision->DoSensorFocusAction(sensor_focus_action, device_id);
    std::cout << "-- Place an object in the center of the lower right quadrant of the camera image, observe the object gets into focus --" << std::endl;
    example_wait_for_focus_action();
}

/**
 * Example showing how to set the intrinsic parameters of the Color and Depth sensors
 */
void my_set_intrinsics(k_api::VisionConfig::VisionConfigClient* vision, uint32_t device_id)
{
    k_api::VisionConfig::IntrinsicParameters intrinsics_old;
    k_api::VisionConfig::IntrinsicParameters intrinsics_new;
    k_api::VisionConfig::IntrinsicParameters intrinsics_reply;
    k_api::VisionConfig::IntrinsicProfileIdentifier profile_id;

    std::cout << "\n\n** Example showing how to set the intrinsic parameters of the Color and Depth sensors **" << std::endl;

    std::cout << "\n-- Using Vision Config Service to get current intrinsic parameters for color resolution 640x480 --" << std::endl;
    profile_id.set_sensor(k_api::VisionConfig::SENSOR_COLOR);
    profile_id.set_resolution(k_api::VisionConfig::RESOLUTION_640x480);
    intrinsics_old = vision->GetIntrinsicParametersProfile(profile_id, device_id);
    print_intrinsic_parameters(intrinsics_old);

    std::cout << "\n-- Using Vision Config Service to set new intrinsic parameters for color resolution 640x480 --" << std::endl;
    intrinsics_new.set_sensor(profile_id.sensor());
    intrinsics_new.set_resolution(profile_id.resolution());
    intrinsics_new.set_principal_point_x(640 / 2 + 0.123456f);
    intrinsics_new.set_principal_point_y(480 / 2 + 1.789012f);
    intrinsics_new.set_focal_length_x(650.567890f);
    intrinsics_new.set_focal_length_y(651.112233f);
    intrinsics_new.mutable_distortion_coeffs()->set_k1(0.2f);
    intrinsics_new.mutable_distortion_coeffs()->set_k2(0.05f);
    intrinsics_new.mutable_distortion_coeffs()->set_p1(1.2f);
    intrinsics_new.mutable_distortion_coeffs()->set_p2(0.999999f);
    intrinsics_new.mutable_distortion_coeffs()->set_k3(0.001f);
    vision->SetIntrinsicParameters(intrinsics_new, device_id);

    std::cout << "\n-- Using Vision Config Service to get new intrinsic parameters for color resolution 640x480 --" << std::endl;
    intrinsics_reply = vision->GetIntrinsicParametersProfile(profile_id, device_id);
    print_intrinsic_parameters(intrinsics_reply);

    std::cout << "\n-- Using Vision Config Service to set back old intrinsic parameters for color resolution 640x480 --" << std::endl;
    vision->SetIntrinsicParameters(intrinsics_old, device_id);


    std::cout << "\n-- Using Vision Config Service to get current intrinsic parameters for depth resolution 424x240 --" << std::endl;
    profile_id.set_sensor(k_api::VisionConfig::SENSOR_DEPTH);
    profile_id.set_resolution(k_api::VisionConfig::RESOLUTION_424x240);
    intrinsics_old = vision->GetIntrinsicParametersProfile(profile_id, device_id);
    print_intrinsic_parameters(intrinsics_old);

    std::cout << "\n-- Using Vision Config Service to set new intrinsic parameters for depth resolution 424x240 --" << std::endl;
    intrinsics_new.set_sensor(profile_id.sensor());
    intrinsics_new.set_resolution(profile_id.resolution());
    intrinsics_new.set_principal_point_x(424 / 2 + 0.123456f);
    intrinsics_new.set_principal_point_y(240 / 2 + 1.789012f);
    intrinsics_new.set_focal_length_x(315.567890f);
    intrinsics_new.set_focal_length_y(317.112233f);
    intrinsics_new.mutable_distortion_coeffs()->set_k1(0.425f);
    intrinsics_new.mutable_distortion_coeffs()->set_k2(1.735102f);
    intrinsics_new.mutable_distortion_coeffs()->set_p1(0.1452f);
    intrinsics_new.mutable_distortion_coeffs()->set_p2(0.767574f);
    intrinsics_new.mutable_distortion_coeffs()->set_k3(2.345678f);
    vision->SetIntrinsicParameters(intrinsics_new, device_id);

    std::cout << "\n-- Using Vision Config Service to get new intrinsic parameters for depth resolution 424x240 --" << std::endl;
    intrinsics_reply = vision->GetIntrinsicParametersProfile(profile_id, device_id);
    print_intrinsic_parameters(intrinsics_reply);

    std::cout << "\n-- Using Vision Config Service to set back old intrinsic parameters for depth resolution 424x240 --" << std::endl;
    vision->SetIntrinsicParameters(intrinsics_old, device_id);
}
