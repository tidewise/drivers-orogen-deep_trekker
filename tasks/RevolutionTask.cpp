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
    queryNewDeviceStateInfo();

    receiveDeviceStateInfo();

    evaluateCameraHeadCommand();

    evaluateGrabberCommand();

    evaluatePositionAndLightCommand();

    evaluatePoweredReelControlCommand();

    RevolutionTaskBase::updateHook();
}

void RevolutionTask::evaluateCameraHeadCommand()
{
    TiltCameraHeadCommand tilt_camera_head_command;
    if (_tilt_camera_head_command.read(tilt_camera_head_command) == RTT::NoData) {
        return;
    }

    CameraHeadCommand camera_head_command;
    if (_camera_head_command.read(camera_head_command) != RTT::NewData) {
        return;
    }

    string address = mDevicesMacAddress.revolution;
    string control_command = mMessageParser.parseTiltCameraHeadCommandMessage(mAPIVersion,
        address,
        camera_head_command,
        tilt_camera_head_command);
    sendRawDataOutput(control_command);
}

void RevolutionTask::evaluateGrabberCommand()
{
    GrabberCommand grabber_command;
    if (_grabber_command.read(grabber_command) != RTT::NewData) {
        return;
    }

    string address = mDevicesMacAddress.revolution;
    string control_command =
        mMessageParser.parseGrabberCommandMessage(mAPIVersion, address, grabber_command);
    sendRawDataOutput(control_command);
}

void RevolutionTask::evaluatePositionAndLightCommand()
{
    PositionAndLightCommand rov2ref_command;
    if (_rov2ref_command.read(rov2ref_command) != RTT::NewData) {
        return;
    }
    string address = mDevicesMacAddress.revolution;
    string control_command = mMessageParser.parseRevolutionCommandMessage(mAPIVersion,
        address,
        rov2ref_command);
    sendRawDataOutput(control_command);
}

void RevolutionTask::evaluatePoweredReelControlCommand()
{
    PoweredReelControlCommand powered_reel_command;
    if (_powered_reel_command.read(powered_reel_command) != RTT::NewData) {
        return;
    }

    string address = mDevicesMacAddress.powered_reel;
    string control_command = mMessageParser.parsePoweredReelCommandMessage(mAPIVersion,
        address,
        powered_reel_command);
    sendRawDataOutput(control_command);
}

void RevolutionTask::sendRawDataOutput(string control_command)
{
    vector<uint8_t> new_data(control_command.begin(), control_command.end());
    RawPacket data_out;
    data_out.time = Time::now();
    data_out.data.resize(new_data.size());
    data_out.data = new_data;
    _data_out.write(data_out);
    // debug output
    _data_command_out.write(data_out);
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
    // debug output
    _data_query_out.write(data_out);
}

void RevolutionTask::receiveDeviceStateInfo()
{
    RawPacket data_in;
    if (_data_in.read(data_in) != RTT::NewData) {
        return;
    }

    string error;
    string data_str(data_in.data.begin(), data_in.data.end());
    if (!mMessageParser.parseJSONMessage(data_str.c_str(), error)) {
        throw invalid_argument(error);
    }
    if (!mDevicesMacAddress.revolution.empty()) {
        _revolution_states.write(getRevolutionStates());
        _revolution_body_states.write(getRevolutionBodyStates());
        _revolution_motor_states.write(getRevolutionMotorStates());
        _grabber_motor_states.write(getGrabberMotorStates());
    }

    if (!mDevicesMacAddress.powered_reel.empty()) {
        _powered_reel_states.write(getPoweredReelStates());
        _powered_reel_motor_states.write(getPoweredReelMotorStates());
    }

    if (!mDevicesMacAddress.manual_reel.empty()) {
        _manual_reel_states.write(getManualReelStates());
    }
}

Revolution RevolutionTask::getRevolutionStates()
{
    Revolution revolution;
    if (!mMessageParser.checkDeviceMacAddress(mDevicesMacAddress.revolution)) {
        throw invalid_argument("Revolution mac address incorrect");
    }
    string rev_address = mDevicesMacAddress.revolution;
    revolution.usage_time = mMessageParser.getTimeUsage(rev_address);
    revolution.vehicle_control = mMessageParser.getRevolutionControlStates(rev_address);
    revolution.left_battery = mMessageParser.getBatteryStates(rev_address, "leftBattery");
    revolution.right_battery =
        mMessageParser.getBatteryStates(rev_address, "rightBattery");
    revolution.light = mMessageParser.getLightIntensity(rev_address);
    revolution.grabber = mMessageParser.getGrabberMotorOvercurrentStates(rev_address);
    revolution.camera_head = mMessageParser.getCameraHeadStates(rev_address);

    return revolution;
}

RevolutionBodyStates RevolutionTask::getRevolutionBodyStates()
{
    if (!mMessageParser.checkDeviceMacAddress(mDevicesMacAddress.revolution)) {
        throw invalid_argument("Revolution mac address incorrect");
    }
    string rev_address = mDevicesMacAddress.revolution;

    return mMessageParser.getRevolutionBodyStates(rev_address);
}

RevolutionMotorStates RevolutionTask::getRevolutionMotorStates()
{
    if (!mMessageParser.checkDeviceMacAddress(mDevicesMacAddress.revolution)) {
        throw invalid_argument("Revolution mac address incorrect");
    }
    string rev_address = mDevicesMacAddress.revolution;

    return mMessageParser.getRevolutionMotorStates(rev_address);
}

PoweredReel RevolutionTask::getPoweredReelStates()
{
    PoweredReel powered_reel;
    if (!mMessageParser.checkDeviceMacAddress(mDevicesMacAddress.powered_reel)) {
        throw invalid_argument("Powered reel mac address incorrect");
    }
    string pwr_reel = mDevicesMacAddress.powered_reel;
    powered_reel.calibrated = mMessageParser.isCalibrated(pwr_reel);
    powered_reel.ready = mMessageParser.isReady(pwr_reel);
    powered_reel.tether_lenght = mMessageParser.getTetherLenght(pwr_reel);
    powered_reel.leak = mMessageParser.isLeaking(pwr_reel);
    powered_reel.cpu_temperature = mMessageParser.getCpuTemperature(pwr_reel);
    powered_reel.battery_1 = mMessageParser.getBatteryStates(pwr_reel, "battery1");
    powered_reel.battery_2 = mMessageParser.getBatteryStates(pwr_reel, "battery2");
    powered_reel.ac_power_connected = mMessageParser.isACPowerConnected(pwr_reel);
    powered_reel.estop_enabled = mMessageParser.isEStopEnabled(pwr_reel);

    return powered_reel;
}

PoweredReelMotorStates RevolutionTask::getPoweredReelMotorStates()
{
    if (!mMessageParser.checkDeviceMacAddress(mDevicesMacAddress.powered_reel)) {
        throw invalid_argument("Powered reel mac address incorrect");
    }
    string pwr_reel = mDevicesMacAddress.powered_reel;

    return mMessageParser.getPoweredReelMotorState(pwr_reel);
}

GrabberMotorStates RevolutionTask::getGrabberMotorStates()
{
    if (!mMessageParser.checkDeviceMacAddress(mDevicesMacAddress.revolution)) {
        throw invalid_argument("Revolution mac address incorrect");
    }
    string rev_address = mDevicesMacAddress.revolution;

    return mMessageParser.getGrabberMotorStates(rev_address);
}

ManualReel RevolutionTask::getManualReelStates()
{
    ManualReel manual_reel;
    if (!mMessageParser.checkDeviceMacAddress(mDevicesMacAddress.manual_reel)) {
        throw invalid_argument("Manual reel mac address incorrect");
    }
    string man_reel = mDevicesMacAddress.manual_reel;
    manual_reel.calibrated = mMessageParser.isCalibrated(man_reel);
    manual_reel.ready = mMessageParser.isReady(man_reel);
    manual_reel.tether_lenght = mMessageParser.getTetherLenght(man_reel);
    manual_reel.leak = mMessageParser.isLeaking(man_reel);
    manual_reel.cpu_temperature = mMessageParser.getCpuTemperature(man_reel);

    return manual_reel;
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
