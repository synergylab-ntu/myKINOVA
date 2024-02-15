

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>

#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/UrdfFactory.h>
#include <rl/mdl/XmlFactory.h>
#include <boost/lexical_cast.hpp>


#include <iostream>
#include <fstream>
#include <math.h>
#include <string>
#include <vector>
#include <conio.h>
#include <time.h>
#include <ctime>

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif

constexpr auto TIMEOUT_DURATION = std::chrono::seconds(20);