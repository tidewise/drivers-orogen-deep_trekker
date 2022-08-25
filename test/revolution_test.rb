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

    def expected_revolution_raw_data(cmd)
        cmd = JSON.dump(
            {
                "apiVersion": "0.10.3",
                "method": "SET",
                "payload": {
                    "devices": {
                        "57B974C0A269": {
                            "auxLights": cmd.light,
                            "control": {
                                "setpoint": {
                                    "pose": {
                                        "localFrame": {
                                            "x": cmd.vehicle_setpoint.position[0],
                                            "y": cmd.vehicle_setpoint.position[1],
                                            "z": cmd.vehicle_setpoint.position[2],
                                            "yaw": cmd.vehicle_setpoint.yaw.rad
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        )
        raw_cmd = Types.iodrivers_base.RawPacket.new
        raw_cmd.data = cmd.each_char.map(&:ord)

        raw_cmd
    end

    def expected_camera_head_raw_data(cmd)
        cmd = JSON.dump(
            {
                "apiVersion": "0.10.3",
                "method": "SET",
                "payload": {
                    "devices": {
                        "57B974C0A269": {
                            "cameraHead": {
                                "lights": cmd.light,
                                "lasers": cmd.laser,
                                "tilt": cmd.speed.elements[0].speed,
                                "camera": {
                                    "exposure": cmd.camera.exposure,
                                    "brightness": cmd.camera.brightness,
                                    "focus": cmd.camera.focus,
                                    "saturation": cmd.camera.saturation,
                                    "sharpness": cmd.camera.sharpness,
                                    "zoom": {
                                        "ratio": cmd.camera.zoom.ratio,
                                        "speed": cmd.camera.zoom.speed.elements[0].speed
                                    }
                                }
                            }
                        }
                    }
                }
            }
        )
        raw_cmd = Types.iodrivers_base.RawPacket.new
        raw_cmd.data = cmd.each_char.map(&:ord)

        raw_cmd
    end

    def expected_grabber_raw_data(cmd)
        cmd = JSON.dump(
            {
                "apiVersion": "0.10.3",
                "method": "SET",
                "payload": {
                    "devices": {
                        "57B974C0A269": {
                            "grabber": {
                                "openClose": cmd.open,
                                "rotate": cmd.speed.elements[0].speed
                            }
                        }
                    }
                }
            }
        )
        raw_cmd = Types.iodrivers_base.RawPacket.new
        raw_cmd.data = cmd.each_char.map(&:ord)

        raw_cmd
    end

    def expected_powered_reel_raw_data(cmd)
        cmd = JSON.dump(
            {
                "apiVersion": "0.10.3",
                "method": "SET",
                "payload": {
                    "devices": {
                        "63B234C0A269": {
                            "reelFoward": cmd.reel_forward,
                            "reelReverse": cmd.reel_reverse,
                            "speed": cmd.speed.elements[0].speed
                        }
                    }
                }
            }
        )
        raw_cmd = Types.iodrivers_base.RawPacket.new
        raw_cmd.data = cmd.each_char.map(&:ord)

        raw_cmd
    end

    it "sends revolution command" do
        raw_cmd = raw_packet_input
        # cmd action input
        cmd_action = Types.deep_trekker.DevicesCommandAction
        cmd_action = "RevolutionCommandAction"
        # revolution cmd input
        cmd = Types.deep_trekker.PositionAndLightCommand.new
        cmd.light = 60
        cmd.vehicle_setpoint.position = Eigen::Vector3.new(1, 3, 4)
        cmd.vehicle_setpoint.yaw.rad = 0.2

        sample = expect_execution do
            syskit_write task.data_in_port, raw_cmd
            syskit_write task.command_action_port, cmd_action
            syskit_write task.rov2ref_command_port, cmd
        end.to do
            have_one_new_sample task.data_out_port
        end

        expected_raw_cmd = expected_revolution_raw_data(cmd)

        json_from_sample = JSON.parse(sample.data.to_byte_array[8..-1])
        json_from_expected = JSON.parse(expected_raw_cmd.data.to_byte_array[8..-1])

        assert_equal json_from_sample["payload"]["devices"]["57B974C0A269"]["auxLights"],\
                     json_from_expected["payload"]["devices"]["57B974C0A269"]["auxLights"]
        assert_equal json_from_sample["payload"]["devices"]["57B974C0A269"]["control"] \
            ["setpoint"]["pose"]["localFrame"]["x"], \
                     json_from_expected["payload"]["devices"]["57B974C0A269"]["control"] \
            ["setpoint"]["pose"]["localFrame"]["x"]
        assert_equal json_from_sample["payload"]["devices"]["57B974C0A269"]["control"] \
            ["setpoint"]["pose"]["localFrame"]["y"], \
                     json_from_expected["payload"]["devices"]["57B974C0A269"]["control"] \
            ["setpoint"]["pose"]["localFrame"]["y"]
        assert_equal json_from_sample["payload"]["devices"]["57B974C0A269"]["control"] \
            ["setpoint"]["pose"]["localFrame"]["z"], \
                     json_from_expected["payload"]["devices"]["57B974C0A269"]["control"] \
            ["setpoint"]["pose"]["localFrame"]["z"]
        assert_equal json_from_sample["payload"]["devices"]["57B974C0A269"]["control"] \
            ["setpoint"]["pose"]["localFrame"]["yaw"], \
                     json_from_expected["payload"]["devices"]["57B974C0A269"]["control"] \
            ["setpoint"]["pose"]["localFrame"]["yaw"]
    end

    it "sends tilt camera head command" do
        raw_cmd = raw_packet_input
        # cmd action input
        cmd_action = Types.deep_trekker.DevicesCommandAction
        cmd_action = "TiltCameraHeadCommandAction"
        # camera head cmd input
        cmd = Types.deep_trekker.TiltCameraHeadCommand.new
        cmd.light = 60
        cmd.laser = true
        cmd.speed = Types.base.samples.Joints.new(
            time: Time.now,
            elements: [
                Types.base.JointState.Speed(3.0)
            ]
        )
        camera = Types.deep_trekker.TamronHarrierZoomCameraCommand.new
        camera.exposure = 1
        camera.brightness = 1
        camera.focus = 1
        camera.saturation = 1
        camera.sharpness = 1
        zoom = Types.deep_trekker.ZoomControlCommand.new
        zoom.ratio = 1
        zoom.speed = Types.base.samples.Joints.new(
            time: Time.now,
            elements: [
                Types.base.JointState.Speed(3.0)
            ]
        )
        camera.zoom = zoom
        cmd.camera = camera

        sample = expect_execution do
            syskit_write task.data_in_port, raw_cmd
            syskit_write task.command_action_port, cmd_action
            syskit_write task.camera_head_command_port, cmd
        end.to do
            have_one_new_sample task.data_out_port
        end

        expected_raw_cmd = expected_camera_head_raw_data(cmd)

        json_from_sample = JSON.parse(sample.data.to_byte_array[8..-1])
        json_from_expected = JSON.parse(expected_raw_cmd.data.to_byte_array[8..-1])

        assert_equal json_from_sample["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["lights"], \
                     json_from_expected["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["lights"]
        assert_equal json_from_sample["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["lasers"], \
                     json_from_expected["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["lasers"]
        assert_equal json_from_sample["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["tilt"], \
                     json_from_expected["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["tilt"]
        assert_equal json_from_sample["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["camera"]["exposure"], \
                     json_from_expected["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["camera"]["exposure"]
        assert_equal json_from_sample["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["camera"]["brightness"], \
                     json_from_expected["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["camera"]["brightness"]
        assert_equal json_from_sample["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["camera"]["focus"], \
                     json_from_expected["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["camera"]["focus"]
        assert_equal json_from_sample["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["camera"]["saturation"], \
                     json_from_expected["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["camera"]["saturation"]
        assert_equal json_from_sample["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["camera"]["sharpness"], \
                     json_from_expected["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["camera"]["sharpness"]
        assert_equal json_from_sample["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["camera"]["zoom"]["ratio"], \
                     json_from_expected["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["camera"]["zoom"]["ratio"]
        assert_equal json_from_sample["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["camera"]["zoom"]["speed"], \
                     json_from_expected["payload"]["devices"]["57B974C0A269"] \
            ["cameraHead"]["camera"]["zoom"]["speed"]
    end

    it "sends grabber command" do
        raw_cmd = raw_packet_input
        # cmd action input
        cmd_action = Types.deep_trekker.DevicesCommandAction
        cmd_action = "GrabberCommandAction"
        # grabber cmd input
        cmd = Types.deep_trekker.GrabberCommand.new
        cmd.open = 1
        cmd.speed = Types.base.samples.Joints.new(
            time: Time.now,
            elements: [
                Types.base.JointState.Speed(3.0)
            ]
        )

        sample = expect_execution do
            syskit_write task.data_in_port, raw_cmd
            syskit_write task.command_action_port, cmd_action
            syskit_write task.grabber_command_port, cmd
        end.to do
            have_one_new_sample task.data_out_port
        end

        expected_raw_cmd = expected_grabber_raw_data(cmd)

        json_from_sample = JSON.parse(sample.data.to_byte_array[8..-1])
        json_from_expected = JSON.parse(expected_raw_cmd.data.to_byte_array[8..-1])

        assert_equal json_from_sample["payload"]["devices"]["57B974C0A269"] \
            ["grabber"]["openClose"], json_from_expected["payload"]["devices"] \
            ["57B974C0A269"]["grabber"]["openClose"]
        assert_equal json_from_sample["payload"]["devices"]["57B974C0A269"] \
            ["grabber"]["rotate"], json_from_expected["payload"]["devices"] \
            ["57B974C0A269"]["grabber"]["rotate"]
    end

    it "sends powered reel command" do
        raw_cmd = raw_packet_input
        # cmd action input
        cmd_action = Types.deep_trekker.DevicesCommandAction
        cmd_action = "PoweredReelCommandAction"
        # powered reel cmd input
        cmd = Types.deep_trekker.PoweredReelControlCommand.new
        cmd.reel_forward = true
        cmd.reel_reverse = false
        cmd.speed = Types.base.samples.Joints.new(
            time: Time.now,
            elements: [
                Types.base.JointState.Speed(3.0)
            ]
        )

        sample = expect_execution do
            syskit_write task.data_in_port, raw_cmd
            syskit_write task.command_action_port, cmd_action
            syskit_write task.reel_command_port, cmd
        end.to do
            have_one_new_sample task.data_out_port
        end

        expected_raw_cmd = expected_powered_reel_raw_data(cmd)

        json_from_sample = JSON.parse(sample.data.to_byte_array[8..-1])
        json_from_expected = JSON.parse(expected_raw_cmd.data.to_byte_array[8..-1])

        assert_equal json_from_sample["payload"]["devices"]["63B234C0A269"]\
            ["reelFoward"], json_from_expected["payload"]["devices"]["63B234C0A269"] \
            ["reelFoward"]
        assert_equal json_from_sample["payload"]["devices"]["63B234C0A269"] \
            ["reelReverse"], json_from_expected["payload"]["devices"]["63B234C0A269"] \
            ["reelReverse"]
        assert_equal json_from_sample["payload"]["devices"]["63B234C0A269"] \
            ["speed"], json_from_expected["payload"]["devices"]["63B234C0A269"] \
            ["speed"]
    end

    it "sends query command for new device state info" do
        raw_cmd = raw_packet_input
        # cmd action input
        cmd_action = Types.deep_trekker.DevicesCommandAction
        cmd_action = "QueryDeviceStateInfo"

        sample = expect_execution do
            syskit_write task.data_in_port, raw_cmd
            syskit_write task.command_action_port, cmd_action
        end.to do
            have_one_new_sample task.data_out_port
        end

        json_from_sample = JSON.parse(sample.data.to_byte_array[8..-1])

        assert_equal json_from_sample["apiVersion"], "0.10.3"
        assert_equal json_from_sample["method"], "GET"
        assert_nil json_from_sample["payload"]
    end
end
