#ifndef __FRAME_H__
#define __FRAME_H__

#include "Frame.pb.h"

#define ACTUATORCONFIG_CLIENT_PROTO_AVAILABLE
#define ACTUATORCYCLIC_CLIENT_PROTO_AVAILABLE
#define BASE_CLIENT_PROTO_AVAILABLE
#define BASECYCLIC_CLIENT_PROTO_AVAILABLE
#define COMMON_CLIENT_PROTO_AVAILABLE
#define CONTROLCONFIG_CLIENT_PROTO_AVAILABLE
#define DEVICECONFIG_CLIENT_PROTO_AVAILABLE
#define DEVICEMANAGER_CLIENT_PROTO_AVAILABLE
#define ERRORS_CLIENT_PROTO_AVAILABLE
#define FRAME_CLIENT_PROTO_AVAILABLE
#define GRIPPERCONFIG_CLIENT_PROTO_AVAILABLE
#define GRIPPERCYCLIC_CLIENT_PROTO_AVAILABLE
#define GRIPPERCYCLICMESSAGE_CLIENT_PROTO_AVAILABLE
#define INTERCONNECTCONFIG_CLIENT_PROTO_AVAILABLE
#define INTERCONNECTCYCLIC_CLIENT_PROTO_AVAILABLE
#define INTERCONNECTCYCLICMESSAGE_CLIENT_PROTO_AVAILABLE
#define PRODUCTCONFIGURATION_CLIENT_PROTO_AVAILABLE
#define SESSION_CLIENT_PROTO_AVAILABLE
#define TEST_CLIENT_PROTO_AVAILABLE
#define VISIONCONFIG_CLIENT_PROTO_AVAILABLE

#define ACTUATORCONFIG_SERVER_PROTO_AVAILABLE
#define ACTUATORCYCLIC_SERVER_PROTO_AVAILABLE
#define BASE_SERVER_PROTO_AVAILABLE
#define BASECYCLIC_SERVER_PROTO_AVAILABLE
#define COMMON_SERVER_PROTO_AVAILABLE
#define CONTROLCONFIG_SERVER_PROTO_AVAILABLE
#define DEVICECONFIG_SERVER_PROTO_AVAILABLE
#define DEVICEMANAGER_SERVER_PROTO_AVAILABLE
#define ERRORS_SERVER_PROTO_AVAILABLE
#define FRAME_SERVER_PROTO_AVAILABLE
#define GRIPPERCONFIG_SERVER_PROTO_AVAILABLE
#define GRIPPERCYCLIC_SERVER_PROTO_AVAILABLE
#define GRIPPERCYCLICMESSAGE_SERVER_PROTO_AVAILABLE
#define INTERCONNECTCONFIG_SERVER_PROTO_AVAILABLE
#define INTERCONNECTCYCLIC_SERVER_PROTO_AVAILABLE
#define INTERCONNECTCYCLICMESSAGE_SERVER_PROTO_AVAILABLE
#define PRODUCTCONFIGURATION_SERVER_PROTO_AVAILABLE
#define SESSION_SERVER_PROTO_AVAILABLE
#define TEST_SERVER_PROTO_AVAILABLE
#define VISIONCONFIG_SERVER_PROTO_AVAILABLE

namespace Kinova
{
namespace Api
{
	enum ServiceIds
	{
		eIdActuatorConfig = 10,
		eIdActuatorCyclic = 11,
		eIdBase = 2,
		eIdBaseCyclic = 3,
		eIdControlConfig = 16,
		eIdDeviceConfig = 9,
		eIdDeviceManager = 23,
		eIdGripperCyclic = 17,
		eIdInterconnectConfig = 14,
		eIdInterconnectCyclic = 15,
		eIdSession = 1,
		eIdTest = 4080,
		eIdVisionConfig = 5,
	};
}
}

#endif