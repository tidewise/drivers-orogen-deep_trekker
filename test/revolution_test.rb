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

        config = Types.deep_trekker.DevicesMacAddress.new
        config.revolution = "57B974C0A269"
        config.manual_reel = "578954C0A26S"
        config.powered_reel = "63B234C0A269"
        @task.properties.api_version = "0.10.3"
        @task.properties.devices_mac_address = config

        syskit_configure_and_start(@task)
    end

    # revolution msg expected by the deep_trekker
    # queried in the component start step
    def raw_packet_input
        cmd = JSON.dump(
            {
                devices: {
                    "57B974C0A269": {
                        "currentSeconds": 2,
                        "control": {
                            "setpoint": {
                                "pose": {
                                    "localFrame": {
                                        "x": 1,
                                        "y": 2,
                                        "z": -2,
                                        "yaw": 0.2
                                    }
                                }
                            },
                            "current": {
                                "pose": {
                                    "localFrame": {
                                        "x": 1,
                                        "y": 2,
                                        "z": -2,
                                        "yaw": 0.2
                                    }
                                }
                            }
                        },
                        "leftBattery": {
                            "percent": 0.8,
                            "voltage": 1,
                            "charging": 2
                        },
                        "rightBattery": {
                            "percent": 0.8,
                            "voltage": 1,
                            "charging": 2
                        },
                        "frontLeftMotorDiagnostics": {
                            "current": 3,
                            "overcurret": false,
                            "rpm": 5.5
                        },
                        "frontRightMotorDiagnostics": {
                            "current": 3,
                            "overcurret": false,
                            "rpm": 5.5
                        },
                        "rearLeftMotorDiagnostics": {
                            "current": 3,
                            "overcurret": false,
                            "rpm": 5.5
                        },
                        "rearRightMotorDiagnostics": {
                            "current": 3,
                            "overcurret": false,
                            "rpm": 5.5
                        },
                        "verticalLeftMotorDiagnostics": {
                            "current": 3,
                            "overcurret": false,
                            "rpm": 5.5
                        },
                        "verticalRightMotorDiagnostics": {
                            "current": 3,
                            "overcurret": false,
                            "rpm": 5.5
                        },
                        "auxLights": 1,
                        "grabber": {
                            "motorDiagnostics": {
                                "current": 3,
                                "overcurret": false,
                                "rpm": 5.5
                            },
                            "openClose": 1,
                            "rotate": 1.0
                        },
                        "cameraHead": {
                            "lights": 1.5,
                            "lasers": true,
                            "tilt": {
                                "speed": 1.5
                            },
                            "tiltMotorDiagnostics": {
                                "current": 3,
                                "overcurret": false,
                                "rpm": 5.5
                            },
                            "camera": {
                                "focus": {
                                    "value": 1.5
                                },
                                "saturation": 2,
                                "sharpness": 2,
                                "zoom": {
                                    "ration": 1.5,
                                    "speed": 1.5
                                }
                            }
                        }
                    },
                    "578954C0A26S": {
                        "calibrator": true,
                        "ready": true,
                        "distance": 15,
                        "leak": false,
                        "cpuTemp": 10
                    },
                    "63B234C0A269": {
                        "calibrator": true,
                        "ready": true,
                        "distance": 15,
                        "leak": false,
                        "cpuTemp": 10,
                        "battery1": {
                            "percent": 0.8,
                            "voltage": 1,
                            "charging": 2
                        },
                        "battery2": {
                            "percent": 0.8,
                            "voltage": 1,
                            "charging": 2
                        },
                        "motor1Diagnostics": {
                            "current": 3,
                            "overcurret": false,
                            "rpm": 5.5
                        },
                        "motor2Diagnostics": {
                            "current": 3,
                            "overcurret": false,
                            "rpm": 5.5
                        },
                        "speed": 5.5,
                        "acConnected": false,
                        "eStop": false
                    }
                }
            }
        )
        raw_cmd = Types.iodrivers_base.RawPacket.new
        raw_cmd.time = Time.now
        raw_cmd.data = cmd.each_char.map(&:ord)

        raw_cmd
    end

    it "sends revolution command" do
        raw_cmd = raw_packet_input
        # revolution cmd input
        cmd = Types.base.LinearAngular6DCommand.new
        cmd.zero!
        cmd.linear = Eigen::Vector3.new(1, 3, 4)
        cmd.angular = Eigen::Vector3.new(0, 0, 0.2)

        sample = expect_execution do
            syskit_write task.light_command_port, 0.2
            syskit_write task.rov2ref_command_port, cmd
            syskit_write task.data_in_port, raw_cmd
        end.to do
            have_one_new_sample task.data_command_out_port
        end

        json_from_sample = JSON.parse(sample.data.to_byte_array[8..-1])

        aux_light = json_from_sample["payload"]["devices"]["57B974C0A269"]["auxLights"]
        local_x = json_from_sample["payload"]["devices"]["57B974C0A269"]["control"] \
                                  ["setpoint"]["pose"]["localFrame"]["x"]
        local_y = json_from_sample["payload"]["devices"]["57B974C0A269"]["control"] \
                                  ["setpoint"]["pose"]["localFrame"]["y"]
        local_z = json_from_sample["payload"]["devices"]["57B974C0A269"]["control"] \
                                  ["setpoint"]["pose"]["localFrame"]["z"]
        local_yaw = json_from_sample["payload"]["devices"]["57B974C0A269"]["control"] \
                                    ["setpoint"]["pose"]["localFrame"]["yaw"]

        assert_equal 0.2 * 100, aux_light # x100 before send to deep_treker
        assert_equal 1, local_x
        assert_equal 3, local_y
        assert_equal 4, local_z
        assert_equal 0.2, local_yaw
    end

    it "sends tilt camera head command" do
        raw_cmd = raw_packet_input
        # camera head cmd input
        cmd_head = Types.deep_trekker.CameraHeadCommand.new
        cmd_head.light = 0.6
        cmd_head.laser = true
        camera = Types.deep_trekker.TamronHarrierZoomCameraCommand.new
        camera.exposure = 1
        camera.brightness = 1
        camera.focus = 0.31
        camera.saturation = 1
        camera.sharpness = 1
        zoom = Types.deep_trekker.ZoomControlCommand.new
        zoom.ratio = 2
        zoom.speed = 0.1
        camera.zoom = zoom
        cmd_head.camera = camera
        # Tilt camera head input
        joint_state = Types.base.JointState.new
        joint_state.speed = 3.0
        joint_state.position = 1.0
        cmd_tilt = Types.base.samples.Joints.new(
            time: Time.now,
            elements: [
                joint_state
            ],
            names: %w[joint]
        )

        sample = expect_execution do
            syskit_write task.data_in_port, raw_cmd
            syskit_write task.camera_head_command_port, cmd_head
            syskit_write task.tilt_camera_head_command_port, cmd_tilt
        end.to do
            have_one_new_sample task.data_command_out_port
        end

        json_from_sample = JSON.parse(sample.data.to_byte_array[8..-1])

        light = json_from_sample["payload"]["devices"]["57B974C0A269"] \
                                ["cameraHead"]["lights"]
        laser = json_from_sample["payload"]["devices"]["57B974C0A269"] \
                                ["cameraHead"]["lasers"]
        tilt_position = json_from_sample["payload"]["devices"]["57B974C0A269"] \
                                        ["cameraHead"]["tilt"]["position"]
        tilt_speed = json_from_sample["payload"]["devices"]["57B974C0A269"] \
                                     ["cameraHead"]["tilt"]["speed"]
        exposure = json_from_sample["payload"]["devices"]["57B974C0A269"] \
                                   ["cameraHead"]["camera"]["exposure"]
        brightness = json_from_sample["payload"]["devices"]["57B974C0A269"] \
                                     ["cameraHead"]["camera"]["brightness"]
        focus = json_from_sample["payload"]["devices"]["57B974C0A269"] \
                                ["cameraHead"]["camera"]["focus"]
        saturation = json_from_sample["payload"]["devices"]["57B974C0A269"] \
                                     ["cameraHead"]["camera"]["saturation"]
        sharpness = json_from_sample["payload"]["devices"]["57B974C0A269"] \
                                    ["cameraHead"]["camera"]["sharpness"]
        ratio = json_from_sample["payload"]["devices"]["57B974C0A269"] \
                                ["cameraHead"]["camera"]["zoom"]["ratio"]
        speed = json_from_sample["payload"]["devices"]["57B974C0A269"] \
                                ["cameraHead"]["camera"]["zoom"]["speed"]

        assert_equal 0.6 * 100, light # x100 before send to deep_treker
        assert_equal true, laser
        assert_equal 1.0, tilt_position
        assert_equal 1.0 * 100, tilt_speed # x100 before send to deep_treker
        assert_equal 1 * 15, exposure # x15 before send to deep_treker
        assert_equal 1 * 100, brightness # x100 before send to deep_treker
        assert_in_delta 0.31 * 100, focus, 0.001 # x100 before send to deep_treker
        assert_equal 1 * 100, saturation # x100 before send to deep_treker
        assert_equal 1 * 100, sharpness # x100 before send to deep_treker
        assert_equal 2, ratio
        assert_in_delta 0.1 * 100, speed, 0.001 # x100 before send to deep_treker
    end

    it "sends grabber command" do
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
            syskit_write task.data_in_port, raw_cmd
            syskit_write task.grabber_command_port, cmd
        end.to do
            have_one_new_sample task.data_command_out_port
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
        raw_cmd = raw_packet_input
        # powered reel cmd input
        cmd = Types.base.samples.Joints.new(
            time: Time.now,
            elements: [
                Types.base.JointState.Speed(-0.25)
            ],
            names: %w[joint]
        )

        sample = expect_execution do
            syskit_write task.data_in_port, raw_cmd
            syskit_write task.powered_reel_command_port, cmd
        end.to do
            have_one_new_sample task.data_command_out_port
        end

        json_from_sample = JSON.parse(sample.data.to_byte_array[8..-1])

        reel_forward = json_from_sample["payload"]["devices"]["63B234C0A269"]\
                                       ["reelFoward"]
        reel_reverse = json_from_sample["payload"]["devices"]["63B234C0A269"] \
                                       ["reelReverse"]
        speed = json_from_sample["payload"]["devices"]["63B234C0A269"]["speed"]

        assert_equal false, reel_forward
        assert_equal true, reel_reverse
        assert_equal -0.25 * 100, speed # x100 before send to deep_treker
    end

    it "sends query command for new device state info" do
        raw_cmd = raw_packet_input

        sample = expect_execution do
            syskit_write task.data_in_port, raw_cmd
        end.to do
            have_one_new_sample task.data_query_out_port
        end

        json_from_sample = JSON.parse(sample.data.to_byte_array[8..-1])

        assert_equal json_from_sample["apiVersion"], "0.10.3"
        assert_equal json_from_sample["method"], "GET"
        assert_nil json_from_sample["payload"]
    end
end
