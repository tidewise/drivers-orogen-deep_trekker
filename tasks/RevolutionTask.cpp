/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RevolutionTask.hpp"

using namespace std;
using namespace base;
using namespace deep_trekker;
using namespace iodrivers_base;

RevolutionTask::RevolutionTask(std::string const& name)
    : RevolutionTaskBase(name)
{
}

RevolutionTask::~RevolutionTask()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See RevolutionTask.hpp for more detailed
// documentation about them.

bool RevolutionTask::configureHook()
{
    if (!RevolutionTaskBase::configureHook()) {
        return false;
    }
    mDevicesMacAddress = _devices_mac_address.get();
    mAPIVersion = _api_version.get();

    return true;
}
bool RevolutionTask::startHook()
{
    if (!RevolutionTaskBase::startHook()) {
        return false;
    }

    // Query device info first
    queryNewDeviceStateInfo();

    return true;
}
void RevolutionTask::updateHook()
{
    // For new device state info,
    // query it by the command action
    receiveDeviceStateInfo();

    DevicesCommandAction command_action;
    if (_command_action.read(command_action) == RTT::NoData) {
        return;
    }

    string control_command;
    switch (command_action) {
        case TiltCameraHeadCommandAction: {
            TiltCameraHeadCommand camera_head_command;
            if (_camera_head_command.read(camera_head_command) != RTT::NewData) {
                return;
            }
            string address = mDevicesMacAddress.revolution;
            control_command =
                mMessageParser.parseTiltCameraHeadCommandMessage(mAPIVersion,
                    address,
                    camera_head_command);
            break;
        }
        case GrabberCommandAction: {
            GrabberCommand grabber_command;
            if (_grabber_command.read(grabber_command) != RTT::NewData) {
                return;
            }
            string address = mDevicesMacAddress.revolution;
            control_command = mMessageParser.parseGrabberCommandMessage(mAPIVersion,
                address,
                grabber_command);
            break;
        }
        case RevolutionCommandAction: {
            RevolutionControlCommand rov2ref_command;
            if (_rov2ref_command.read(rov2ref_command) != RTT::NewData) {
                return;
            }
            string address = mDevicesMacAddress.revolution;
            control_command = mMessageParser.parseRevolutionCommandMessage(mAPIVersion,
                address,
                rov2ref_command);
            break;
        }
        case PoweredReelCommandAction: {
            PoweredReelControlCommand reel_command;
            if (_reel_command.read(reel_command) != RTT::NewData) {
                return;
            }
            string address = mDevicesMacAddress.powered_reel;
            control_command = mMessageParser.parsePoweredReelCommandMessage(mAPIVersion,
                address,
                reel_command);
            break;
        }
        case QueryDeviceStateInfo: {
            queryNewDeviceStateInfo();
            return;
        }
        default:
            break;
    }

    vector<uint8_t> new_data(control_command.begin(), control_command.end());
    RawPacket data_out;
    data_out.time = Time::now();
    data_out.data.resize(new_data.size());
    data_out.data = new_data;
    _data_out.write(data_out);

    RevolutionTaskBase::updateHook();
}

void RevolutionTask::queryNewDeviceStateInfo()
{
    mMessageParser = CommandAndStateMessageParser();
    string get_message = mMessageParser.parseGetMessage(mAPIVersion);
    vector<uint8_t> new_data(get_message.begin(), get_message.end());
    RawPacket data_out;
    data_out.time = Time::now();
    data_out.data.resize(new_data.size());
    data_out.data = new_data;
    _data_out.write(data_out);
}

void RevolutionTask::receiveDeviceStateInfo()
{
    RawPacket data_in;
    if (_data_in.read(data_in) == RTT::NoData) {
        return;
    }

    string error;
    string data_str(data_in.data.begin(), data_in.data.end());
    if (!mMessageParser.parseJSONMessage(data_str.c_str(), error)) {
        throw invalid_argument(error);
    }
    _devices_info.write(getDevicesInfo());
}

DevicesInfo RevolutionTask::getDevicesInfo()
{
    Revolution revolution;
    if (!mMessageParser.checkDeviceMacAddress(mDevicesMacAddress.revolution)) {
        throw invalid_argument("Revolution mac address incorrect");
    }
    string rev_address = mDevicesMacAddress.revolution;
    revolution.usage_time = mMessageParser.getTimeUsage(rev_address);
    revolution.vehicle_control = mMessageParser.getVehicleStates(rev_address);
    revolution.left_battery = mMessageParser.getBatteryInfo(rev_address, "leftBattery");
    revolution.right_battery = mMessageParser.getBatteryInfo(rev_address, "rightBattery");
    revolution.front_left_motor =
        mMessageParser.getMotorInfo(rev_address, "frontLeftMotorDiagnostics");
    revolution.front_right_motor =
        mMessageParser.getMotorInfo(rev_address, "frontRightMotorDiagnostics");
    revolution.rear_left_motor =
        mMessageParser.getMotorInfo(rev_address, "rearLeftMotorDiagnostics");
    revolution.rear_right_motor =
        mMessageParser.getMotorInfo(rev_address, "rearRightMotorDiagnostics");
    revolution.vertical_left_motor =
        mMessageParser.getMotorInfo(rev_address, "verticalLeftMotorDiagnostics");
    revolution.vertical_right_motor =
        mMessageParser.getMotorInfo(rev_address, "verticalRightMotorDiagnostics");
    revolution.light = mMessageParser.getLightInfo(rev_address);
    revolution.grabber = mMessageParser.getGrabberMotorInfo(rev_address);
    revolution.camera_head = mMessageParser.getCameraHeadInfo(rev_address);

    ManualReel manual_reel;
    if (!mMessageParser.checkDeviceMacAddress(mDevicesMacAddress.manual_reel)) {
        throw invalid_argument("Manual reel mac address incorrect");
    }
    string man_reel = mDevicesMacAddress.manual_reel;
    manual_reel.calibrated = mMessageParser.getCalibrateInfo(man_reel);
    manual_reel.ready = mMessageParser.getReadyInfo(man_reel);
    manual_reel.tether_distance = mMessageParser.getTetherDistanceInfo(man_reel);
    manual_reel.leak = mMessageParser.getLeakInfo(man_reel);
    manual_reel.cpu_temperature = mMessageParser.getTemperatureInfo(man_reel);

    PoweredReel powered_reel;
    if (!mMessageParser.checkDeviceMacAddress(mDevicesMacAddress.powered_reel)) {
        throw invalid_argument("Powered reel mac address incorrect");
    }
    string pwr_reel = mDevicesMacAddress.powered_reel;
    powered_reel.calibrated = mMessageParser.getCalibrateInfo(pwr_reel);
    powered_reel.ready = mMessageParser.getReadyInfo(pwr_reel);
    powered_reel.tether_distance = mMessageParser.getTetherDistanceInfo(pwr_reel);
    powered_reel.leak = mMessageParser.getLeakInfo(pwr_reel);
    powered_reel.cpu_temperature = mMessageParser.getTemperatureInfo(pwr_reel);
    powered_reel.battery_1 = mMessageParser.getBatteryInfo(pwr_reel, "battery1");
    powered_reel.battery_2 = mMessageParser.getBatteryInfo(pwr_reel, "battery2");
    powered_reel.motor_1 = mMessageParser.getMotorInfo(pwr_reel, "motor1Diagnostics");
    powered_reel.motor_2 = mMessageParser.getMotorInfo(pwr_reel, "motor2Diagnostics");
    powered_reel.speed = mMessageParser.getSpeedInfo(pwr_reel);
    powered_reel.ac_power_connected = mMessageParser.getACPowerInfo(pwr_reel);
    powered_reel.estop_enabled = mMessageParser.getEStopInfo(pwr_reel);

    DevicesInfo device;
    device.revolution = revolution;
    device.manual_reel = manual_reel;
    device.powered_reel = powered_reel;

    return device;
}

void RevolutionTask::errorHook()
{
    RevolutionTaskBase::errorHook();
}
void RevolutionTask::stopHook()
{
    RevolutionTaskBase::stopHook();
}
void RevolutionTask::cleanupHook()
{
    RevolutionTaskBase::cleanupHook();
}
