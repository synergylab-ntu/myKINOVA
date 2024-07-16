// Reset RGB and D resolutions to maximum
// Disable autofocus from RGB
// Set focus distance midway for RGB
// Set focus point mid-image for RGB

#include <myKINOVA_VISION.h>
#include <myKINOVA_includes.h>		// baked in
#include <myKINOVA_LOGGING.h>



int main(int argc, char** argv)
{
    auto parsed_args = myParseExampleArguments(argc, argv);

    // Create API objects
    auto error_callback = [](k_api::KError err) { cout << "_________ callback error _________" << err.toString(); };
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(parsed_args.ip_address, PORT);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(parsed_args.username);
    create_session_info.set_password(parsed_args.password);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating session for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    std::cout << "Session created" << std::endl;

    // Create services
    auto vision = new k_api::VisionConfig::VisionConfigClient(router);
    auto device_manager = new k_api::DeviceManager::DeviceManagerClient(router);

    // Example core
    example_display_usage();
    uint32_t vision_device_id = example_vision_get_device_id(device_manager);
    if (vision_device_id != 0)
    {
        //std::cout << "######################################################################################### Intrinsics and extrinsics before update" << std::endl;
        //my_print_INTRINSICS_EXTRINSICS(vision, vision_device_id);

        //example_routed_vision_set_intrinsics(vision, vision_device_id);
        //std::cout << "######################################################################################### Intrinsics and extrinsics after update" << std::endl;
        //my_print_INTRINSICS_EXTRINSICS(vision, vision_device_id);

        //my_set_resolution_1920x1080_480x270(vision, vision_device_id);
        my_disable_focus(vision, vision_device_id);
        my_set_focus_1920x1080(vision, vision_device_id, 0.5f, 0.5f); // focus point at the center of image
        my_set_focus_distance(vision, vision_device_id);

        //std::cout << "Intrinsics and extrinsics after update" << std::endl;
        

        //
        
        //if (logfile.is_open())
        //{
         //   
        //}

        my_print_INTRINSICS_EXTRINSICS(vision, vision_device_id);
        
        //my_autofocus_ON(vision, vision_device_id);

        //example_routed_vision_set_extrinsics(vision, vision_device_id);
        /*while (!(GetKeyState('Q') & 0x8000))
        {
            float x_fraction, y_fraction;
            std::cout << "Enter x_fraction" << std::endl;
            std::cin >> x_fraction;
            std::cout << "Enter y_fraction" << std::endl;
            std::cin >> y_fraction;

            std::cout << "Keep pressing Q if you want to abort" << std::endl;
            my_set_focus(vision, vision_device_id, x_fraction, y_fraction);

        }*/
    }
    else
    {
        std::cout << "Error : No Vision module found!" << std::endl;
    }

    // Close API session
    session_manager->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();

    // Destroy the API
    delete vision;
    delete device_manager;
    delete session_manager;
    delete router;
    delete transport;
}

