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
            upper: 2.62, lower: -1.91
        )
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
                            "heading": 20,
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
                json["method"] == "SET"
            end
        end

        json_from_sample = JSON.parse(sample.data.to_byte_array[8..-1])

        fwd = json_from_sample["payload"]["devices"]["57B974C0A269"]["drive"]["thrust"] \
                              ["forward"]
        lat = json_from_sample["payload"]["devices"]["57B974C0A269"]["drive"]["thrust"] \
                              ["lateral"]
        vert = json_from_sample["payload"]["devices"]["57B974C0A269"]["drive"]["thrust"] \
                              ["vertical"]
        yaw = json_from_sample["payload"]["devices"]["57B974C0A269"]["drive"]["thrust"] \
                              ["yaw"]

        assert_equal 100, fwd
        assert_equal 30, lat
        assert_equal 40, vert
        assert_equal 20, yaw
    end

    it "sends camera light command" do
        syskit_configure_and_start(@task)
        cmd = 0.6

        sample = expect_execution do
            syskit_write task.camera_head_light_port, cmd
        end.to do
            have_one_new_sample(task.data_out_port).matching do |s|
                json = JSON.parse(s.data.to_byte_array[8..-1])
                json["method"] == "SET"
            end
        end

        json_from_sample = JSON.parse(sample.data.to_byte_array[8..-1])

        light = json_from_sample["payload"]["devices"]["57B974C0A269"] \
                                ["cameraHead"]["light"]["intensity"]
        assert_equal 0.6 * 100, light # x100 before send to deep_treker
    end

    it "sends camera laser command" do
        syskit_configure_and_start(@task)
        laser = true

        sample = expect_execution do
            syskit_write task.camera_head_laser_enable_port, laser
        end.to do
            have_one_new_sample(task.data_out_port).matching do |s|
                json = JSON.parse(s.data.to_byte_array[8..-1])
                json["method"] == "SET"
            end
        end

        json_from_sample = JSON.parse(sample.data.to_byte_array[8..-1])

        laser = json_from_sample["payload"]["devices"]["57B974C0A269"] \
                                ["cameraHead"]["laser"]["enabled"]
        assert_equal true, laser
    end

    it "sends no tilt camera head command when hasnt received a valid position yet" do
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
            have_no_new_sample(task.data_out_port)
        end
    end

    it "sends tilt camera head command when it is at a valid position" do
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

        sample = expect_execution do
            syskit_write task.tilt_camera_head_command_port, cmd_tilt
        end.to do
            have_one_new_sample(task.data_out_port).matching do |s|
                json = JSON.parse(s.data.to_byte_array[8..-1])
                json["method"] == "SET"
            end
        end

        json_from_sample = JSON.parse(sample.data.to_byte_array[8..-1])

        tilt_speed = json_from_sample["payload"]["devices"]["57B974C0A269"] \
                                     ["cameraHead"]["tilt"]["speed"]

        assert_equal 100, tilt_speed
    end

    it "doesnt send tilt camera head command when it would go past one of the limits" do
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
            have_no_new_sample(task.data_out_port)
        end

        cmd = JSON.parse(raw_cmd.data.to_byte_array[8..-1])
        cmd["payload"]["devices"]["57B974C0A269"]["cameraHead"]["tilt"]["position"] = -110
        raw_cmd.data = JSON.dump(cmd).each_char.map(&:ord)

        expect_execution do
            syskit_write task.data_in_port, raw_cmd
        end.to do
            have_one_new_sample(task.camera_head_tilt_states_port)
        end

        # Tilt camera head input
        joint_state = Types.base.JointState.new
        joint_state.speed = -0.5
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
            have_no_new_sample(task.data_out_port)
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
                json["method"] == "SET"
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

        sample = expect_execution do
            syskit_write task.powered_reel_command_port, cmd
        end.to do
            have_one_new_sample(task.data_out_port).matching do |s|
                json = JSON.parse(s.data.to_byte_array[8..-1])
                json["method"] == "SET"
            end
        end

        json_from_sample = JSON.parse(sample.data.to_byte_array[8..-1])

        speed = json_from_sample["payload"]["devices"]["63B234C0A269"]["speed"]

        assert_equal -25, speed
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
        json_head = json_from_raw["payload"]["devices"]["57B974C0A269"] \
                                 ["cameraHead"]
        json_camera = json_from_raw["payload"]["devices"]["57B974C0A269"] \
                                   ["cameraHead"]["camera"]

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
        assert_equal sample.drive_modes.auto_stabilization, 1
        assert_equal sample.drive_modes.motors_disabled, 1
        assert_equal sample.cpu_temperature, 43
    end
end
