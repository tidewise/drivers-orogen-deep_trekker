/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RevolutionTask.hpp"
#include <algorithm>
#include <base-logging/Logging.hpp>
using namespace std;
using namespace base;
using namespace deep_trekker;
using namespace iodrivers_base;

RevolutionTask::RevolutionTask(std::string const& name)
    : RevolutionTaskBase(name)
    , m_camera_head_tilt_position(base::unknown<double>())
{
    _nwu_magnetic2nwu.set(base::Angle::fromRad(0));
}

RevolutionTask::~RevolutionTask()
{
}

void tryParseAndWriteIgnoringExceptions(function<void()> write_func)
{
    try {
        write_func();
    }
    catch (invalid_argument& e) {
        LOG_ERROR(e.what());
    }
}

bool RevolutionTask::configureHook()
{
    if (!RevolutionTaskBase::configureHook()) {
        return false;
    }
    m_api_version = _api_version.get();
    m_devices_model = _devices_model.get();
    m_devices_id = _devices_id.get();
    m_input_timeout = _input_timeout.get();
    m_camera_head_limits = _camera_head_limits.get();
    m_nwu_magnetic2nwu_ori =
        Eigen::AngleAxisd(_nwu_magnetic2nwu.get().getRad(), Eigen::Vector3d::UnitZ());

    // Warning: By default, the storage order in Eigen is column-major.
    // This means that it is in effect transposed on load when compared
    // with how it looks in YAML, that is the YAML
    //
    // thrusterMatrix:
    // data: [1, 2, 3,
    //        4, 5, 6]
    //
    // Will actually be loaded in thrusterMatrix as row(0) = [1, 4], row(1) = [2, 5] ...
    //
    // The transpose here makes sure that one that writes YAML config files gets what
    // (s)he expects.
    m_compensation_matrix = _compensation_matrix.get().transpose();

    GetRequestConfig powered_reel_get_request;
    powered_reel_get_request.update_interval = _powered_reel_update_interval.get();
    if (!powered_reel_get_request.update_interval.isNull()) {
        powered_reel_get_request.trigger_deadline = Time::now();
        powered_reel_get_request.request =
            m_message_parser.getRequestForPoweredReelStates(m_api_version,
                m_devices_id.powered_reel);
    }
    GetRequestConfig revolution_pose_z_attitude_request;
    revolution_pose_z_attitude_request.update_interval =
        _pose_z_attitude_update_interval.get();
    if (!revolution_pose_z_attitude_request.update_interval.isNull()) {
        revolution_pose_z_attitude_request.trigger_deadline = Time::now();
        revolution_pose_z_attitude_request.request =
            m_message_parser.getRequestForRevolutionPoseZAttitude(m_api_version,
                m_devices_id.revolution);
    }

    m_get_requests.push_back(powered_reel_get_request);
    m_get_requests.push_back(revolution_pose_z_attitude_request);

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

    sendGetRequests(m_get_requests);

    if (!m_devices_id.revolution.empty()) {
        evaluateCameraHeadLightCommand();
        evaluateCameraHeadLaserCommand();
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

void RevolutionTask::sendGetRequests(vector<GetRequestConfig>& requests)
{
    for (auto& req : requests) {
        if (!req.update_interval.isNull() && Time::now() > req.trigger_deadline) {
            sendRawDataOutput(req.request);
            req.trigger_deadline = Time::now() + req.update_interval;
        }
    }
}

base::commands::LinearAngular6DCommand RevolutionTask::compensateDriveCommand(
    base::commands::LinearAngular6DCommand const& drive_command)
{
    // Some of these will be always NAN, so we should correct for it.
    base::commands::LinearAngular6DCommand corrected_drive_command;
    for (unsigned int i = 0; i < 3; i++)
    {
        corrected_drive_command.linear[i] = isUnknown(drive_command.linear[i]) ? 0 : drive_command.linear[i];
        corrected_drive_command.angular[i] = isUnknown(drive_command.angular[i]) ? 0 : drive_command.angular[i];
    }
    Eigen::MatrixXd drive_matrix(7, 1);
    drive_matrix << corrected_drive_command.linear.x(),
        corrected_drive_command.linear.y(), corrected_drive_command.linear.z(),
        corrected_drive_command.angular.x(), corrected_drive_command.angular.y(),
        corrected_drive_command.angular.z(), 1.0;
    base::Vector6d compensated_drive_matrix;
    compensated_drive_matrix = m_compensation_matrix * drive_matrix;
    base::commands::LinearAngular6DCommand compensated_drive;
    compensated_drive.linear = compensated_drive_matrix.head(3);
    compensated_drive.angular = compensated_drive_matrix.tail(3);
    compensated_drive.time = Time::now();
    return compensated_drive;
}

void RevolutionTask::evaluateDriveCommand()
{
    base::commands::LinearAngular6DCommand drive;
    auto port_state = _drive_command.read(drive);
    auto now = Time::now();
    if (port_state == RTT::NewData) {
        m_deadlines.drive = now + m_input_timeout;
    }
    else if (port_state == RTT::NoData) {
        string disable_auto_stabilization =
            m_message_parser.parseAutoStabilizationRevolutionCommandMessage(m_api_version,
                m_devices_id.revolution,
                m_devices_model.revolution,
                false);
        sendRawDataOutput(disable_auto_stabilization);
        return;
    }

    if (now > m_deadlines.drive) {
        return;
    }

    string enable_auto_stabilization =
        m_message_parser.parseAutoStabilizationRevolutionCommandMessage(m_api_version,
            m_devices_id.revolution,
            m_devices_model.revolution,
            true);
    sendRawDataOutput(enable_auto_stabilization);

    m_compensated_command = compensateDriveCommand(drive);

    string drive_command =
        m_message_parser.parseDriveRevolutionCommandMessage(m_api_version,
            m_devices_id.revolution,
            m_devices_model.revolution,
            m_compensated_command);
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

void RevolutionTask::evaluateMotorsDisabledCommand()
{
    bool motors_disabled;
    if (_motors_disabled.read(motors_disabled) == RTT::NoData) {
        return;
    }

    string command =
        m_message_parser.parseMotorsDisabledRevolutionCommandMessage(m_api_version,
            m_devices_id.revolution,
            m_devices_model.revolution,
            motors_disabled);
    sendRawDataOutput(command);
}

void RevolutionTask::evaluateTiltCameraHeadCommand()
{
    samples::Joints tilt_camera_head_command;
    auto port_state = _tilt_camera_head_command.read(tilt_camera_head_command);
    // Register the time once, allowing for an input timeout of 0
    Time now = Time::now();
    if (port_state == RTT::NewData) {
        m_deadlines.tilt_camera_head = now + m_input_timeout;
    }
    else if (port_state == RTT::NoData) {
        return;
    }

    if (now > m_deadlines.tilt_camera_head) {
        return;
    }

    if (isUnknown(m_camera_head_tilt_position)) {
        tilt_camera_head_command.elements[0].speed = 0;
    }
    if (m_camera_head_tilt_position >= m_camera_head_limits.upper &&
        tilt_camera_head_command.elements[0].speed > 0) {
        tilt_camera_head_command.elements[0].speed = 0;
    }
    if (m_camera_head_tilt_position <= m_camera_head_limits.lower &&
        tilt_camera_head_command.elements[0].speed < 0) {
        tilt_camera_head_command.elements[0].speed = 0;
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

void RevolutionTask::evaluateCameraHeadLightCommand()
{
    double camera_head_light;
    if (_camera_head_light.read(camera_head_light) == RTT::NoData) {
        return;
    }

    string address = m_devices_id.revolution;
    string control_command = m_message_parser.parseCameraHeadLightMessage(m_api_version,
        address,
        m_devices_model.revolution,
        m_devices_model.camera_head,
        camera_head_light);
    sendRawDataOutput(control_command);
}

void RevolutionTask::evaluateCameraHeadLaserCommand()
{
    bool laser_enabled;
    if (_camera_head_laser_enable.read(laser_enabled) == RTT::NoData) {
        return;
    }

    string address = m_devices_id.revolution;
    string control_command = m_message_parser.parseCameraHeadLaserMessage(m_api_version,
        address,
        m_devices_model.revolution,
        m_devices_model.camera_head,
        laser_enabled);
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
    auto now = Time::now();
    if (port_state == RTT::NewData) {
        m_deadlines.powered_reel = now + m_input_timeout;
    }
    else if (port_state == RTT::NoData) {
        return;
    }

    if (now > m_deadlines.powered_reel) {
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
        tryParseAndWriteIgnoringExceptions(
            [&]() { _revolution_states.write(getRevolutionStates()); });
        tryParseAndWriteIgnoringExceptions(
            [&]() { _motors_disabled_state.write(getRevolutionMotorsDisabled()); });
        tryParseAndWriteIgnoringExceptions(
            [&]() { _auto_stabilization_state.write(getRevolutionAutoStabilization()); });
        tryParseAndWriteIgnoringExceptions(
            [&]() { _camera_head_states.write(getCameraHeadStates()); });
        tryParseAndWriteIgnoringExceptions(
            [&]() { _revolution_motor_states.write(getRevolutionMotorStates()); });
        tryParseAndWriteIgnoringExceptions(
            [&]() { _camera_head_tilt_states.write(getCameraHeadTiltMotorState()); });
        tryParseAndWriteIgnoringExceptions([&]() {
            _camera_head_tilt_states_rbs.write(getCameraHeadTiltMotorStateRBS());
        });
        tryParseAndWriteIgnoringExceptions(
            [&]() { _grabber_motor_states.write(getGrabberMotorStates()); });
        tryParseAndWriteIgnoringExceptions(
            [&]() { _revolution_pose_z_attitude.write(getRevolutionPoseZAttitude()); });
    }

    if (!m_devices_id.powered_reel.empty()) {
        tryParseAndWriteIgnoringExceptions([&]() {
            auto state = getPoweredReelStates();
            _tether_length.write(state.tether_length);
            _powered_reel_states.write(state);
        });
        tryParseAndWriteIgnoringExceptions(
            [&]() { _powered_reel_motor_states.write(getPoweredReelMotorStates()); });
    }

    if (!m_devices_id.manual_reel.empty()) {
        tryParseAndWriteIgnoringExceptions(
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
    revolution.compensated_command = m_compensated_command;

    revolution.left_battery =
        m_message_parser.getBatteryStates(rev_address, "leftBattery");
    revolution.right_battery =
        m_message_parser.getBatteryStates(rev_address, "rightBattery");

    revolution.aux_light = m_message_parser.getAuxLightIntensity(rev_address);
    revolution.cameras = m_message_parser.getCameras(rev_address);
    revolution.cpu_temperature = m_message_parser.getCpuTemperature(rev_address);
    revolution.drive_modes = m_message_parser.getRevolutionDriveModes(rev_address);
    revolution.time = Time::now();

    return revolution;
}

TiltCameraHead RevolutionTask::getCameraHeadStates()
{
    return m_message_parser.getCameraHeadStates(m_devices_id.revolution);
}

samples::RigidBodyState RevolutionTask::getRevolutionPoseZAttitude()
{
    auto rbs = m_message_parser.getRevolutionPoseZAttitude(m_devices_id.revolution);
    auto rov2nwu_magnetic_ori = rbs.orientation;
    rbs.orientation = m_nwu_magnetic2nwu_ori * rov2nwu_magnetic_ori;
    rbs.sourceFrame = "body";
    rbs.targetFrame = "world";
    return rbs;
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

samples::RigidBodyState RevolutionTask::getCameraHeadTiltMotorStateRBS()
{
    return m_message_parser.getCameraHeadTiltMotorStateRBS(m_devices_id.revolution);
}

bool RevolutionTask::getRevolutionAutoStabilization()
{
    return m_message_parser.getRevolutionAutoStabilization(m_devices_id.revolution);
}

bool RevolutionTask::getRevolutionMotorsDisabled()
{
    return m_message_parser.getRevolutionMotorsDisabled(m_devices_id.revolution);
}

PoweredReel RevolutionTask::getPoweredReelStates()
{
    string pwr_reel = m_devices_id.powered_reel;

    PoweredReel powered_reel;
    powered_reel.time = Time::now();
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
