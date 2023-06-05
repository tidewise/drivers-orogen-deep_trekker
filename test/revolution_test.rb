# frozen_string_literal: true

using_task_library "deep_trekker"

import_types_from "base"
import_types_from "deep_trekker"
import_types_from "iodrivers_base"

require "json"

describe OroGen.deep_trekker.RevolutionTask do
    run_live

    attr_reader :task

    before do
        @task = syskit_deploy(
            OroGen.deep_trekker
                  .RevolutionTask
                  .deployed_as("revolution")
        )

        config = Types.deep_trekker.DevicesModel.new
        config.revolution = 13
        config.powered_reel = 108
        ids = Types.deep_trekker.DevicesID.new
        ids.revolution = "57B974C0A269"
        ids.powered_reel = "63B234C0A269"

        @task.properties.api_version = "0.10.3"
        @task.properties.devices_model = config
        @task.properties.devices_id = ids
        @task.properties.input_timeout = Time.at(1)
        @task.properties.camera_head_limits = Types.deep_trekker.CameraHeadLimits.new(
            upper: 2.44, lower: -1.91
        )
        compensation_matrix_transposed = Eigen::MatrixX.new(7, 6)
        # from_a is column-major, so the the first 7 elements of the array
        # will be the first column
        compensation_matrix_transposed.from_a(
            [1, 0, 0, 0, 0, 0, 0,
             0, 1.1, 0, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0, -0.1,
             0, 0, 0, 1, 0, 0, 0,
             0, 0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 0, 1, 0]
        )
        @task.properties.compensation_matrix = compensation_matrix_transposed
    end

    # revolution msg expected by the deep_trekker
    # queried in the component start step
    def raw_packet_input
        cmd = JSON.dump(
            {
                payload: {
                    devices: {
                        "57B974C0A269": {
                            "auxLight": { "intensity": 10 },
                            "bodyLeak": false,
                            "cameraHead": {
                                "cpuTemp": 26,
                                "ip": "10.1.1.1",
                                "lasers": { "enabled": false },
                                "leak": false,
                                "light": { "intensity": 5 },
                                "model": 102,
                                "tilt": {
                                    "position": 167.2
                                },
                                "tiltMotorDiagnostics": {
                                    "current": 0,
                                    "rpm": 10,
                                    "pwm": 42,
                                    "overcurrent": false
                                },
                                "usageTime": { "currentSeconds": 500, "totalSeconds": 10 }
                            },
                            "cameras": {
                                "31028309sac": {
                                    "ip": "10.1.1.2",
                                    "model": 5,
                                    "osd": { "enabled": true },
                                    "streams": {
                                        "blabal13123al": {
                                            "active": true
                                        }
                                    },
                                    "type": "MAIN"
                                }
                            },
                            "cpuTemp": 43,
                            "depth": 20,
                            "drive": {
                                "modes": {
                                    "altitudeLock": false,
                                    "autoStabilization": true,
                                    "depthLock": false,
                                    "headingLock": false,
                                    "motorsDisabled": true
                                }
                            },
                            "frontLeftMotorDiagnostics": {
                                "current": 3,
                                "overcurrent": false,
                                "pwm": 20,
                                "rpm": 5.5
                            },
                            "frontRightMotorDiagnostics": {
                                "current": 3,
                                "overcurrent": false,
                                "pwm": 20,
                                "rpm": 5.5
                            },
                            "grabber": {
                                "openCloseMotorDiagnostics": {
                                    "current": 8,
                                    "overcurrent": true,
                                    "rpm": 5.5,
                                    "pwm": 20
                                },
                                "rotateMotorDiagnostics": {
                                    "current": 3,
                                    "overcurrent": false,
                                    "pwm": 20,
                                    "rpm": 5.5
                                }
                            },
                            "heading": 30,
                            "leftBattery": {
                                "percent": 80,
                                "voltage": 50,
                                "charging": true
                            },
                            "model": 13,
                            "rearLeftMotorDiagnostics": {
                                "current": 3,
                                "overcurrent": true,
                                "rpm": 5.5
                            },
                            "rearRightMotorDiagnostics": {
                                "current": 3,
                                "overcurrent": false,
                                "rpm": 5.5
                            },
                            "rightBattery": {
                                "percent": 80,
                                "voltage": 60,
                                "charging": 20
                            },
                            "roll": 3,
                            "pitch": 5,
                            "usageTime": { "currentSeconds": 20, "totalSeconds": 20 },
                            "verticalLeftMotorDiagnostics": {
                                "current": 3,
                                "overcurrent": true,
                                "rpm": 5.5
                            },
                            "verticalRightMotorDiagnostics": {
                                "current": 3,
                                "overcurrent": true,
                                "rpm": 5.5
                            }
                        },
                        "63B234C0A269": {
                            "acConnected": true,
                            "distance": 100,
                            "leak": false,
                            "cpuTemp": 10,
                            "battery1": {
                                "percent": 80,
                                "voltage": 10,
                                "charging": true
                            },
                            "battery2": {
                                "percent": 80,
                                "voltage": 10,
                                "charging": false
                            },
                            "motor1Diagnostics": {
                                "current": 3,
                                "overcurrent": false,
                                "pwm": 5.5
                            },
                            "motor2Diagnostics": {
                                "current": 3,
                                "overcurrent": false,
                                "pwm": 5.5
                            },
                            "eStop": false
                        }
                    }
                }
            }
        )
        raw_cmd = Types.iodrivers_base.RawPacket.new
        raw_cmd.time = Time.now
        raw_cmd.data = cmd.each_char.map(&:ord)

        raw_cmd
    end

    it "sends drive revolution command" do
        syskit_configure_and_start(@task)
        cmd = Types.base.LinearAngular6DCommand.new
        cmd.zero!
        cmd.linear = Eigen::Vector3.new(1, 0.3, 0.4)
        cmd.angular = Eigen::Vector3.new(0, 0, 0.2)

        sample = expect_execution do
            syskit_write task.drive_command_port, cmd
        end.to do
            have_one_new_sample(task.data_out_port).matching do |s|
                json = JSON.parse(s.data.to_byte_array[8..-1])
                next if json["method"] != "SET"

                json["payload"]["devices"]["57B974C0A269"]["drive"]["thrust"]
            end
        end

        json = JSON.parse(sample.data.to_byte_array[8..-1])

        fwd = json["payload"]["devices"]["57B974C0A269"]["drive"]["thrust"]["forward"]
        lat = json["payload"]["devices"]["57B974C0A269"]["drive"]["thrust"]["lateral"]
        vert = json["payload"]["devices"]["57B974C0A269"]["drive"]["thrust"]["vertical"]
        yaw = json["payload"]["devices"]["57B974C0A269"]["drive"]["thrust"]["yaw"]

        assert_equal 100, fwd
        assert_equal -33, lat
        assert_equal -30, vert
        assert_equal -20, yaw
    end

    it "sends drive revolution command and turns auto stabilization" do
        syskit_configure_and_start(@task)
        cmd = Types.base.LinearAngular6DCommand.new
        cmd.zero!
        cmd.linear = Eigen::Vector3.new(1, 0.3, 0.4)
        cmd.angular = Eigen::Vector3.new(0, 0, 0.2)

        sample = expect_execution do
            syskit_write task.drive_command_port, cmd
        end.to do
            [
                have_one_new_sample(task.data_out_port).matching do |s|
                    json = JSON.parse(s.data.to_byte_array[8..-1])
                    next if json["method"] != "SET"

                    json["payload"]["devices"]["57B974C0A269"]["drive"]["thrust"]
                end,
                have_one_new_sample(task.data_out_port).matching do |s|
                    json = JSON.parse(s.data.to_byte_array[8..-1])
                    next if json["method"] != "SET"

                    modes = json["payload"]["devices"]["57B974C0A269"]["drive"]["modes"]
                    modes["autoStabilization"] == true
                end
            ]
        end

        json = JSON.parse(sample[0].data.to_byte_array[8..-1])

        fwd = json["payload"]["devices"]["57B974C0A269"]["drive"]["thrust"] \
                              ["forward"]
        lat = json["payload"]["devices"]["57B974C0A269"]["drive"]["thrust"] \
                              ["lateral"]
        vert = json["payload"]["devices"]["57B974C0A269"]["drive"]["thrust"] \
                              ["vertical"]
        yaw = json["payload"]["devices"]["57B974C0A269"]["drive"]["thrust"] \
                              ["yaw"]

        assert_equal 100, fwd
        assert_equal -33, lat
        assert_equal -30, vert
        assert_equal -20, yaw
    end

    it "turns auto stabilization off when the drive command port is not connected" do
        syskit_configure_and_start(@task)

        expect_execution do
            syskit_write task.light_command_port, 0.4
        end.to do
            have_one_new_sample(task.data_out_port).matching do |s|
                json = JSON.parse(s.data.to_byte_array[8..-1])
                next unless json["method"] == "SET"

                drive = json["payload"]["devices"]["57B974C0A269"]["drive"]
                next unless drive

                drive["modes"]["autoStabilization"] == false
            end
        end
    end

    it "sends auxiliary light command" do
        syskit_configure_and_start(@task)
        cmd = 0.4

        expect_execution do
            syskit_write task.light_command_port, cmd
        end.to do
            have_one_new_sample(task.data_out_port).matching do |s|
                json = JSON.parse(s.data.to_byte_array[8..-1])
                next unless json["method"] == "SET"
                next unless json["payload"]["devices"]["57B974C0A269"]["auxLight"]

                aux_light = json["payload"]["devices"]["57B974C0A269"]["auxLight"]
                aux_light["intensity"] == 40
            end
        end
    end

    it "sends camera light command" do
        syskit_configure_and_start(@task)
        cmd = 0.6

        expect_execution do
            syskit_write task.camera_head_light_port, cmd
        end.to do
            have_one_new_sample(task.data_out_port).matching do |s|
                json = JSON.parse(s.data.to_byte_array[8..-1])
                next unless json["method"] == "SET"
                next unless json["payload"]["devices"]["57B974C0A269"]["cameraHead"]

                camera_head = json["payload"]["devices"]["57B974C0A269"]["cameraHead"]
                camera_head["light"]["intensity"] == 0.6 * 100
            end
        end
    end

    it "sends camera laser command" do
        syskit_configure_and_start(@task)
        laser = true

        expect_execution do
            syskit_write task.camera_head_laser_enable_port, laser
        end.to do
            have_one_new_sample(task.data_out_port).matching do |s|
                json = JSON.parse(s.data.to_byte_array[8..-1])
                next unless json["method"] == "SET"
                next unless json["payload"]["devices"]["57B974C0A269"]["cameraHead"]

                json["payload"]["devices"]["57B974C0A269"] \
                    ["cameraHead"]["laser"]["enabled"] == true
            end
        end
    end

    it "sends zero speed camera head command when hasnt received a valid position yet" do
        syskit_configure_and_start(@task)

        # Tilt camera head input
        joint_state = Types.base.JointState.new
        joint_state.speed = 3.0
        cmd_tilt = Types.base.samples.Joints.new(
            time: Time.now,
            elements: [
                joint_state
            ],
            names: %w[joint]
        )

        expect_execution do
            syskit_write task.tilt_camera_head_command_port, cmd_tilt
        end.to do
            have_one_new_sample(task.data_out_port).matching do |s|
                json = JSON.parse(s.data.to_byte_array[8..-1])
                next unless json["method"] == "SET"

                camera_head = json["payload"]["devices"]["57B974C0A269"]["cameraHead"]
                next unless camera_head

                camera_head["tilt"]["speed"] == 0
            end
        end
    end

    it "sends a tilt camera head command when has received a valid position" do
        syskit_configure_and_start(@task)

        raw_cmd = raw_packet_input
        cmd = JSON.parse(raw_cmd.data.to_byte_array[8..-1])
        cmd["payload"]["devices"]["57B974C0A269"]["cameraHead"]["tilt"]["position"] = 90
        raw_cmd.data = JSON.dump(cmd).each_char.map(&:ord)

        expect_execution do
            syskit_write task.data_in_port, raw_cmd
        end.to do
            have_one_new_sample(task.camera_head_tilt_states_port)
        end

        # Tilt camera head input
        joint_state = Types.base.JointState.new
        joint_state.speed = 0.8
        cmd_tilt = Types.base.samples.Joints.new(
            time: Time.now,
            elements: [
                joint_state
            ],
            names: %w[joint]
        )

        expect_execution do
            syskit_write task.tilt_camera_head_command_port, cmd_tilt
        end.to do
            have_one_new_sample(task.data_out_port).matching do |s|
                json = JSON.parse(s.data.to_byte_array[8..-1])
                camera_head = json["payload"]["devices"]["57B974C0A269"]["cameraHead"]
                next unless camera_head

                camera_head["tilt"]["speed"] == 80
            end
        end
    end

    it "sends at most 100 tilt camera head command when it is above the threshold" do
        syskit_configure_and_start(@task)

        raw_cmd = raw_packet_input
        cmd = JSON.parse(raw_cmd.data.to_byte_array[8..-1])
        cmd["payload"]["devices"]["57B974C0A269"]["cameraHead"]["tilt"]["position"] = 90
        raw_cmd.data = JSON.dump(cmd).each_char.map(&:ord)

        expect_execution do
            syskit_write task.data_in_port, raw_cmd
        end.to do
            have_one_new_sample(task.camera_head_tilt_states_port)
        end

        # Tilt camera head input
        joint_state = Types.base.JointState.new
        joint_state.speed = 3.0
        cmd_tilt = Types.base.samples.Joints.new(
            time: Time.now,
            elements: [
                joint_state
            ],
            names: %w[joint]
        )

        expect_execution do
            syskit_write task.tilt_camera_head_command_port, cmd_tilt
        end.to do
            have_one_new_sample(task.data_out_port).matching do |s|
                json = JSON.parse(s.data.to_byte_array[8..-1])
                camera_head = json["payload"]["devices"]["57B974C0A269"]["cameraHead"]
                next unless camera_head

                camera_head = json["payload"]["devices"]["57B974C0A269"]["cameraHead"]
                camera_head["tilt"]["speed"] == 100
            end
        end
    end

    it "send tilt camera head command with 0 speed when it would go past " \
       "one of the limits" do
        # Sometimes one of the previously issued tilt commands would still be repeating
        # as we wrote a new tilt command due to the input timeout being 1s. This would
        # result in a different sample than the one we expect. Setting this to no input
        # timeout for this test.
        @task.properties.input_timeout = Time.at(0)
        syskit_configure_and_start(@task)

        raw_cmd = raw_packet_input
        cmd = JSON.parse(raw_cmd.data.to_byte_array[8..-1])
        cmd["payload"]["devices"]["57B974C0A269"]["cameraHead"]["tilt"]["position"] = 150
        raw_cmd.data = JSON.dump(cmd).each_char.map(&:ord)

        expect_execution do
            syskit_write task.data_in_port, raw_cmd
        end.to do
            have_one_new_sample(task.camera_head_tilt_states_port)
        end

        # Tilt camera head input
        joint_state = Types.base.JointState.new
        joint_state.speed = 0.5
        cmd_tilt = Types.base.samples.Joints.new(
            time: Time.now,
            elements: [
                joint_state
            ],
            names: %w[joint]
        )

        expect_execution do
            syskit_write task.tilt_camera_head_command_port, cmd_tilt
        end.to do
            have_one_new_sample(task.data_out_port).matching do |s|
                json = JSON.parse(s.data.to_byte_array[8..-1])
                next unless json["method"] == "SET"

                camera_head = json["payload"]["devices"]["57B974C0A269"]["cameraHead"]
                next unless camera_head

                camera_head["tilt"]["speed"] == 0
            end
        end

        cmd = JSON.parse(raw_cmd.data.to_byte_array[8..-1])
        cmd["payload"]["devices"]["57B974C0A269"]["cameraHead"]["tilt"]["position"] = -115
        raw_cmd.data = JSON.dump(cmd).each_char.map(&:ord)

        expect_execution do
            syskit_write task.data_in_port, raw_cmd
        end.to do
            have_one_new_sample(task.camera_head_tilt_states_port)
        end

        # Tilt camera head input
        joint_state = Types.base.JointState.new
        joint_state.speed = -0.5
        cmd_tilt.elements = [joint_state]

        expect_execution do
            syskit_write task.tilt_camera_head_command_port, cmd_tilt
        end.to do
            have_one_new_sample(task.data_out_port).matching do |s|
                json = JSON.parse(s.data.to_byte_array[8..-1])
                next unless json["method"] == "SET"

                camera_head = json["payload"]["devices"]["57B974C0A269"]["cameraHead"]
                next unless camera_head

                camera_head["tilt"]["speed"] == 0
            end
        end
    end

    it "sends grabber command" do
        syskit_configure_and_start(@task)
        raw_cmd = raw_packet_input
        # grabber cmd input
        open_close = Types.base.JointState.new
        open_close.raw = 1
        rotate = Types.base.JointState.new
        rotate.raw = -0.2
        cmd = Types.base.samples.Joints.new(
            time: Time.now,
            elements: [
                open_close,
                rotate
            ],
            names: %w[open_close rotate]
        )

        sample = expect_execution do
            syskit_write task.grabber_command_port, cmd
        end.to do
            have_one_new_sample(task.data_out_port).matching do |s|
                json = JSON.parse(s.data.to_byte_array[8..-1])
                next unless json["method"] == "SET"

                json["payload"]["devices"]["57B974C0A269"]["grabber"]
            end
        end

        json_from_sample = JSON.parse(sample.data.to_byte_array[8..-1])
        open_close_cmd = json_from_sample["payload"]["devices"]["57B974C0A269"] \
                                         ["grabber"]["openClose"]
        rotate_cmd = json_from_sample["payload"]["devices"]["57B974C0A269"] \
                                     ["grabber"]["rotate"]

        assert_in_delta 1 * 100, open_close_cmd, 0.01 # x100 before send to deep_treker
        assert_in_delta -0.2 * 100, rotate_cmd, 0.01 # x100 before send to deep_treker
    end

    it "sends powered reel command" do
        syskit_configure_and_start(@task)
        cmd = Types.base.samples.Joints.new(
            time: Time.now,
            elements: [
                Types.base.JointState.Speed(-0.25)
            ],
            names: %w[joint]
        )

        expect_execution do
            syskit_write task.powered_reel_command_port, cmd
        end.to do
            have_one_new_sample(task.data_out_port).matching do |s|
                json = JSON.parse(s.data.to_byte_array[8..-1])
                next unless json["method"] == "SET"
                next unless json["payload"]["devices"]["63B234C0A269"]

                json["payload"]["devices"]["63B234C0A269"]["speed"] == -25
            end
        end
    end

    it "outputs revolution states" do
        syskit_configure_and_start(@task)
        raw_cmd = raw_packet_input

        sample = expect_execution do
            syskit_write task.data_in_port, raw_cmd
        end.to do
            have_one_new_sample task.revolution_states_port
        end

        json_from_raw = JSON.parse(raw_cmd.data.to_byte_array[8..-1])

        json_root = json_from_raw["payload"]["devices"]["57B974C0A269"]
        json_aux_light = json_from_raw["payload"]["devices"]["57B974C0A269"] \
                                  ["auxLight"]["intensity"]

        assert_equal sample.aux_light, json_aux_light / 100.0
        assert_equal sample.usage_time.tv_sec, json_root["usageTime"]["currentSeconds"]
        assert_equal sample.front_right_motor_overcurrent, 0
        assert_equal sample.front_left_motor_overcurrent, 0
        assert_equal sample.rear_right_motor_overcurrent, 0
        assert_equal sample.rear_left_motor_overcurrent, 1
        assert_equal sample.vertical_right_motor_overcurrent, 1
        assert_equal sample.vertical_left_motor_overcurrent, 1
        assert_in_delta sample.left_battery.charge,
                        json_root["leftBattery"]["percent"] / 100.0, 0.01
        assert_in_delta sample.right_battery.charge,
                        json_root["rightBattery"]["percent"] / 100.0, 0.01
        assert_equal sample.drive_modes.altitude_lock, 0
        assert_equal sample.drive_modes.heading_lock, 0
        assert_equal sample.drive_modes.depth_lock, 0
        assert_equal sample.cpu_temperature, 43
    end

    it "outputs the ROV's orientation + Z" do
        syskit_configure_and_start(@task)
        raw_cmd = raw_packet_input

        sample = expect_execution do
            syskit_write task.data_in_port, raw_cmd
        end.to do
            have_one_new_sample task.revolution_pose_z_attitude_port
        end

        assert sample.position.x.nan?
        assert sample.position.y.nan?
        assert_in_delta(-20, sample.position.z)
        assert_in_delta 3 * Math::PI / 180, sample.orientation.roll
        assert_in_delta 5 * Math::PI / 180, sample.orientation.pitch
        assert_in_delta -30 * Math::PI / 180, sample.orientation.yaw
    end

    it "applies the magnetic declination correction" do
        @task.properties.nwu_magnetic2nwu = { rad: 0.2 }
        syskit_configure_and_start(@task)
        raw_cmd = raw_packet_input

        sample = expect_execution do
            syskit_write task.data_in_port, raw_cmd
        end.to do
            have_one_new_sample task.revolution_pose_z_attitude_port
        end

        assert sample.position.x.nan?
        assert sample.position.y.nan?
        assert_in_delta(-20, sample.position.z)
        assert_in_delta 3 * Math::PI / 180, sample.orientation.roll
        assert_in_delta 5 * Math::PI / 180, sample.orientation.pitch
        assert_in_delta(0.2 - 30 * Math::PI / 180, sample.orientation.yaw)
    end
end
