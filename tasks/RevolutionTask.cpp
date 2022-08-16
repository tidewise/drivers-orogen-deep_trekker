/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RevolutionTask.hpp"

using namespace std;
using namespace base;
using namespace deep_trekker;
using namespace iodrivers_base;

RevolutionTask::RevolutionTask(std::string const& name) : RevolutionTaskBase(name) {}

RevolutionTask::~RevolutionTask() {}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See RevolutionTask.hpp for more detailed
// documentation about them.

bool RevolutionTask::configureHook()
{
    if (!RevolutionTaskBase::configureHook())
    {
        return false;
    }
    mDevicesMacAddress = _devices_mac_address.get();

    return true;
}
bool RevolutionTask::startHook()
{
    if (!RevolutionTaskBase::startHook())
    {
        return false;
    }

    // Query device info first
    queryDeviceStateInfo();

    return true;
}
void RevolutionTask::updateHook()
{
    RawPacket data_in;
    if (_data_in.read(data_in) == RTT::NoData)
    {
        return;
    }

    string data_diagnostics;
    data_diagnostics.assign(data_in.data.begin(), data_in.data.end());
    
    // TODO - call lib to parser

    Revolution diagnostics;
    _deep_trekker_diagnostics.write(diagnostics);
    

    RoVCommandAction command_action;
    if(_command_action.read(command_action) == RTT::NoData)
    {
        return;
    }

    string command;
    switch (command_action)
    {
    case CAMERA_HEAD_COMMAND:
    {
        TiltCameraHeadCommand camera_head_command;
        if(_camera_head_command.read(camera_head_command) != RTT::NewData)
        {
            return;
        }
        // TODO - call lib to parser
    }
    case ZOOM_CAMERA_COMMAND:
    {
        TamronHarrierZoomCameraCommand zoom_camera_command;
        if(_zoom_camera_command.read(zoom_camera_command) != RTT::NewData)
        {
            return;
        }
        // TODO - call lib to parser
    }
    case GRABBER_COMMAND:
    {
        GrabberCommand grabber_command;
        if(_grabber_command.read(grabber_command) != RTT::NewData)
        {
            return;
        }
        // TODO - call lib to parser
    }
    case LASER_LIGHT_COMMANDS:
    {
        LaserLightCommands laser_light_commands;
        if(_laser_light_commands.read(laser_light_commands) != RTT::NewData)
        {
            return;
        }
        // TODO - call lib to parser
    }
    case ROV_SETPOINT_COMMAND:
    {
        RovControlCommand rov2ref_setpoint;
        if(_rov2ref_setpoint.read(rov2ref_setpoint) != RTT::NewData)
        {
            return;
        }
        // TODO - call lib to parser
    }
    default:
        break;
    }

    // TODO - convert command to rawpacket before send
    RawPacket data_out;
    _data_out.write(data_out);

    RevolutionTaskBase::updateHook();
}

void RevolutionTask::queryDeviceStateInfo()
{
    mMessageParser = CommandAndStateMessageParser();
    string get_message = mMessageParser.parseGetMessage();
    vector<uint8_t> new_data(get_message.begin(), get_message.end());
    RawPacket data_out;
    data_out.time = Time::now();
    data_out.data = new_data;
    _data_out.write(data_out);
}
void RevolutionTask::cleanupHook()
{
    RevolutionTaskBase::cleanupHook();
}
void RevolutionTask::errorHook() { RevolutionTaskBase::errorHook(); }
void RevolutionTask::stopHook() { RevolutionTaskBase::stopHook(); }
