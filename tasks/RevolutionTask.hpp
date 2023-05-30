/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef DEEP_TREKKER_REVOLUTIONTASK_TASK_HPP
#define DEEP_TREKKER_REVOLUTIONTASK_TASK_HPP

#include "base/Float.hpp"
#include "deep_trekker/CommandAndStateMessageParser.hpp"
#include "deep_trekker/DeepTrekkerCommands.hpp"
#include "deep_trekker/DeepTrekkerStates.hpp"
#include "deep_trekker/RevolutionTaskBase.hpp"
#include "iodrivers_base/RawPacket.hpp"

namespace deep_trekker {

    /*! \class RevolutionTask
     * \brief The task context provides and requires services. It uses an ExecutionEngine
     to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These
     interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the
     associated workflow.
     *
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','deep_trekker::RevolutionTask')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix
     argument.
     */
    class RevolutionTask : public RevolutionTaskBase {
        friend class RevolutionTaskBase;

    protected:
    public:
        /** TaskContext constructor for RevolutionTask
         * \param name Name of the task. This name needs to be unique to make it
         * identifiable via nameservices. \param initial_state The initial TaskState of
         * the TaskContext. Default is Stopped state.
         */
        RevolutionTask(std::string const& name = "deep_trekker::RevolutionTask");

        /** Default deconstructor of RevolutionTask
         */
        ~RevolutionTask();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        struct PeriodicPortsDeadline {
            base::Time tilt_camera_head;
            base::Time drive;
            base::Time powered_reel;
        };

        struct GetRequestConfig {
            std::string request;
            base::Time update_interval;
            base::Time trigger_deadline;
        };

    private:
        std::string m_api_version;
        CommandAndStateMessageParser m_message_parser;
        DevicesID m_devices_id;
        DevicesModel m_devices_model;
        base::Time m_input_timeout;
        CameraHeadLimits m_camera_head_limits;
        double m_camera_head_tilt_position;
        PeriodicPortsDeadline m_deadlines;
        std::vector<GetRequestConfig> m_get_requests;
        base::Quaterniond m_nwu_magnetic2nwu_ori;
        base::MatrixXd m_compensation_matrix;

        void receiveDeviceStateInfo();
        DevicesID parseDevicesID(Json::Value const& parsed_data,
            DevicesModel const& model);

        void evaluateCameraHeadLightCommand();
        void evaluateCameraHeadLaserCommand();
        void evaluateTiltCameraHeadCommand();
        void evaluateLightCommand();
        void evaluateGrabberCommand();
        void evaluateDriveModeCommand();
        void evaluateDriveCommand();
        void evaluatePoweredReelControlCommand();
        void sendGetRequests(std::vector<GetRequestConfig>& requests);
        void sendRawDataOutput(std::string control_command);
        Revolution getRevolutionStates();
        TiltCameraHead getCameraHeadStates();
        ManualReel getManualReelStates();
        PoweredReel getPoweredReelStates();
        base::samples::RigidBodyState getRevolutionPoseZAttitude();
        base::samples::Joints getRevolutionMotorStates();
        base::samples::Joints getCameraHeadTiltMotorState();
        base::samples::RigidBodyState getCameraHeadTiltMotorStateRBS();
        base::samples::Joints getPoweredReelMotorStates();
        Grabber getGrabberMotorStates();
        base::commands::LinearAngular6DCommand compensateDriveCommand(
            base::commands::LinearAngular6DCommand const& drive_command);
    };
} // namespace deep_trekker

#endif
