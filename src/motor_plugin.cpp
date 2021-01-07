#include <oskar_driver/motor_plugin.h>

namespace ahhaa_oskar
{
MotorPlugin::MotorPlugin(BoardComms *comms, std::string name) : Plugin(comms, name)
{
}

MotorPlugin::~MotorPlugin()
{
}
}  // namespace ahhaa_oskar