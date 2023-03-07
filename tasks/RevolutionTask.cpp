/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RevolutionTask.hpp"
#include <algorithm>
#include <base-logging/Logging.hpp>

using namespace std;
using namespace base;
using namespace deep_trekker;
using namespace iodrivers_base;

RevolutionTask::RevolutionTask(std::string const& name)
    : RevolutionTaskBase(name), m_camera_head_tilt_position(base::unknown<double>())
{
}

RevolutionTask::~RevolutionTask()
{
}

void tryWriteIgnoringExceptions(function<void()> write_func)
{
    try {
        write_func();
    }
    catch (invalid_argument& e) {
        LOG_ERROR(e.what());
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See RevolutionTask.hpp for more detailed
// documentation about them.

bool RevolutionTask::configureHook()
{
    if (!RevolutionTaskBase::configureHook()) {
        return false;
    }
    m_api_version = _api_version.get();
    m_devices_model = _devices_model.get();
    m_devices_id = _devices_id.get();
    m_input_timeout = _input_timeout.get();

    return true;
}

bool RevolutionTask::startHook()
{
    if (!RevolutionTaskBase::startHook()) {
        return false;
    }

    return true;
}

void RevolutionTask::updateHook()
{
    receiveDeviceStateInfo();

    if (!m_devices_id.revolution.empty()) {
        evaluateCameraHeadCommand();
        evaluateTiltCameraHeadCommand();
        evaluateLightCommand();
        evaluateGrabberCommand();
        evaluateDriveCommand();
        evaluateDriveModeCommand();
    }

    if (!m_devices_id.powered_reel.empty()) {
        evaluatePoweredReelControlCommand();
    }

    RevolutionTaskBase::updateHook();
}

void RevolutionTask::evaluateDriveCommand()
{
    base::commands::LinearAngular6DCommand drive;
    auto port_state = _drive_command.read(drive);
    if (port_state == RTT::NewData) {
        m_deadlines.drive = Time::now() + m_input_timeout;
    }

    if (Time::now() > m_deadlines.drive) {
        return;
    }

    string drive_command =
        m_message_parser.parseDriveRevolutionCommandMessage(m_api_version,
            m_devices_id.revolution,
            m_devices_model.revolution,
            drive);
    sendRawDataOutput(drive_command);
}

void RevolutionTask::evaluateDriveModeCommand()
{
    DriveMode mode;
    if (_drive_mode_command.read(mode) == RTT::NoData) {
        return;
    }

    string mode_command =
        m_message_parser.parseDriveModeRevolutionCommandMessage(m_api_version,
            m_devices_id.revolution,
            m_devices_model.revolution,
            mode);
    sendRawDataOutput(mode_command);
}

void RevolutionTask::evaluateTiltCameraHeadCommand()
{
    samples::Joints tilt_camera_head_command;
    auto port_state = _tilt_camera_head_command.read(tilt_camera_head_command);
    if (port_state == RTT::NewData) {
        m_deadlines.tilt_camera_head = Time::now() + m_input_timeout;
    }

    if (Time::now() > m_deadlines.tilt_camera_head) {
        return;
    }

    string address = m_devices_id.revolution;
    string control_command =
        m_message_parser.parseTiltCameraHeadCommandMessage(m_api_version,
            address,
            m_devices_model.revolution,
            m_devices_model.camera_head,
            tilt_camera_head_command);
    sendRawDataOutput(control_command);
}

void RevolutionTask::evaluateCameraHeadCommand()
{
    CameraHeadCommand camera_head_command;
    if (_camera_head_command.read(camera_head_command) == RTT::NoData) {
        return;
    }

    string address = m_devices_id.revolution;
    string control_command = m_message_parser.parseCameraHeadCommandMessage(m_api_version,
        address,
        m_devices_model.revolution,
        camera_head_command);
    sendRawDataOutput(control_command);
}

void RevolutionTask::evaluateLightCommand()
{
    double intensity;
    if (_light_command.read(intensity) == RTT::NoData) {
        return;
    }

    string address = m_devices_id.revolution;
    string control_command = m_message_parser.parseAuxLightCommandMessage(m_api_version,
        address,
        m_devices_model.revolution,
        intensity);
    sendRawDataOutput(control_command);
}

void RevolutionTask::evaluateGrabberCommand()
{
    samples::Joints grabber_command;
    if (_grabber_command.read(grabber_command) == RTT::NoData) {
        return;
    }

    string address = m_devices_id.revolution;
    string control_command = m_message_parser.parseGrabberCommandMessage(m_api_version,
        address,
        grabber_command);
    sendRawDataOutput(control_command);
}

void RevolutionTask::evaluatePoweredReelControlCommand()
{
    samples::Joints powered_reel_command;
    auto port_state = _powered_reel_command.read(powered_reel_command);
    if (port_state == RTT::NewData) {
        m_deadlines.powered_reel = Time::now() + m_input_timeout;
    }
    if (Time::now() > m_deadlines.powered_reel) {
        return;
    }

    string address = m_devices_id.powered_reel;
    string control_command =
        m_message_parser.parsePoweredReelCommandMessage(m_api_version,
            address,
            m_devices_model.powered_reel,
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
}

void RevolutionTask::receiveDeviceStateInfo()
{
    RawPacket data_in;
    if (_data_in.read(data_in) != RTT::NewData) {
        return;
    }

    string error;
    string data_str(data_in.data.begin(), data_in.data.end());
    if (!m_message_parser.parseJSONMessage(data_str.c_str(), error)) {
        throw invalid_argument(error);
    }

    if (!m_devices_id.revolution.empty()) {
        tryWriteIgnoringExceptions(
            [&]() { _revolution_states.write(getRevolutionStates()); });
        tryWriteIgnoringExceptions(
            [&]() { _camera_head_states.write(getCameraHeadStates()); });
        tryWriteIgnoringExceptions(
            [&]() { _revolution_motor_states.write(getRevolutionMotorStates()); });
        tryWriteIgnoringExceptions(
            [&]() { _camera_head_tilt_states.write(getCameraHeadTiltMotorState()); });
        tryWriteIgnoringExceptions(
            [&]() { _grabber_motor_states.write(getGrabberMotorStates()); });
        tryWriteIgnoringExceptions(
            [&]() { _revolution_pose_z_attitude.write(getRevolutionPoseZAttitude()); });
    }

    if (!m_devices_id.powered_reel.empty()) {
        tryWriteIgnoringExceptions(
            [&]() { _powered_reel_states.write(getPoweredReelStates()); });
        tryWriteIgnoringExceptions(
            [&]() { _powered_reel_motor_states.write(getPoweredReelMotorStates()); });
    }

    if (!m_devices_id.manual_reel.empty()) {
        tryWriteIgnoringExceptions(
            [&]() { _manual_reel_states.write(getManualReelStates()); });
    }
}

Revolution RevolutionTask::getRevolutionStates()
{
    Revolution revolution;

    auto rev_address = m_devices_id.revolution;
    revolution.usage_time = m_message_parser.getTimeUsage(rev_address);
    revolution.front_right_motor_overcurrent =
        m_message_parser.getMotorOvercurrentStates(rev_address,
            "frontRightMotorDiagnostics");
    revolution.front_left_motor_overcurrent =
        m_message_parser.getMotorOvercurrentStates(rev_address,
            "frontLeftMotorDiagnostics");
    revolution.rear_right_motor_overcurrent =
        m_message_parser.getMotorOvercurrentStates(rev_address,
            "rearRightMotorDiagnostics");
    revolution.rear_left_motor_overcurrent =
        m_message_parser.getMotorOvercurrentStates(rev_address,
            "rearLeftMotorDiagnostics");
    revolution.vertical_right_motor_overcurrent =
        m_message_parser.getMotorOvercurrentStates(rev_address,
            "verticalRightMotorDiagnostics");
    revolution.vertical_left_motor_overcurrent =
        m_message_parser.getMotorOvercurrentStates(rev_address,
            "verticalLeftMotorDiagnostics");

    revolution.left_battery =
        m_message_parser.getBatteryStates(rev_address, "leftBattery");
    revolution.right_battery =
        m_message_parser.getBatteryStates(rev_address, "rightBattery");

    revolution.aux_light = m_message_parser.getAuxLightIntensity(rev_address);
    revolution.cameras = m_message_parser.getCameras(rev_address);
    revolution.cpu_temperature = m_message_parser.getCpuTemperature(rev_address);
    revolution.drive_modes = m_message_parser.getRevolutionDriveModes(rev_address);
    revolution.timestamp = Time::now();

    return revolution;
}

TiltCameraHead RevolutionTask::getCameraHeadStates()
{
    return m_message_parser.getCameraHeadStates(m_devices_id.revolution);
}

samples::RigidBodyState RevolutionTask::getRevolutionPoseZAttitude()
{
    return m_message_parser.getRevolutionPoseZAttitude(m_devices_id.revolution);
}

samples::Joints RevolutionTask::getRevolutionMotorStates()
{
    return m_message_parser.getRevolutionMotorStates(m_devices_id.revolution);
}

samples::Joints RevolutionTask::getCameraHeadTiltMotorState()
{
    auto state = m_message_parser.getCameraHeadTiltMotorState(m_devices_id.revolution);
    m_camera_head_tilt_position = state.elements[0].position;
    return state;
}

PoweredReel RevolutionTask::getPoweredReelStates()
{
    string pwr_reel = m_devices_id.powered_reel;

    PoweredReel powered_reel;
    powered_reel.timestamp = Time::now();
    powered_reel.tether_length = m_message_parser.getTetherLength(pwr_reel);
    powered_reel.leak = m_message_parser.isLeaking(pwr_reel);
    powered_reel.cpu_temperature = m_message_parser.getCpuTemperature(pwr_reel);
    powered_reel.battery_1 = m_message_parser.getBatteryStates(pwr_reel, "battery1");
    powered_reel.battery_2 = m_message_parser.getBatteryStates(pwr_reel, "battery2");
    powered_reel.ac_power_connected = m_message_parser.isACPowerConnected(pwr_reel);
    powered_reel.estop_enabled = m_message_parser.isEStopEnabled(pwr_reel);
    powered_reel.motor_1_overcurrent =
        m_message_parser.getMotorOvercurrentStates(pwr_reel, "motor1Diagnostics");
    powered_reel.motor_2_overcurrent =
        m_message_parser.getMotorOvercurrentStates(pwr_reel, "motor2Diagnostics");
    return powered_reel;
}

samples::Joints RevolutionTask::getPoweredReelMotorStates()
{
    return m_message_parser.getPoweredReelMotorState(m_devices_id.powered_reel);
}

Grabber RevolutionTask::getGrabberMotorStates()
{
    return m_message_parser.getGrabberMotorStates(m_devices_id.revolution);
}

ManualReel RevolutionTask::getManualReelStates()
{
    string man_reel = m_devices_id.manual_reel;

    ManualReel manual_reel;
    manual_reel.tether_length = m_message_parser.getTetherLength(man_reel);
    manual_reel.leak = m_message_parser.isLeaking(man_reel);
    manual_reel.cpu_temperature = m_message_parser.getCpuTemperature(man_reel);
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
