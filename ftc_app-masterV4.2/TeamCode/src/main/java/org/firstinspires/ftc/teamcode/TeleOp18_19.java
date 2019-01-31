package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import com.qualcomm.robotcore.hardware.CRServo;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;


import java.util.List;
import java.util.Locale;

/**
 * Created by Grace on 10/25/2018.
 */
@TeleOp (name = "TeleOp Rover Ruckus", group = "Linear Opmode")
public class TeleOp18_19 extends LinearOpMode {

    Hardware h = new Hardware(DcMotor.RunMode.RUN_TO_POSITION);

    int currentChainPos;

    @Override
    public void runOpMode() {

        try {
            h.init(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            e.printStackTrace();
        }


        h.MRGyro.calibrate();
        while(h.MRGyro.isCalibrating())
        {
            telemetry.update();
            telemetry.addData("Gyro:", "calibrating");
        }
        telemetry.addData("Calibration", "complete");
        telemetry.update();


        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        waitForStart();
        h.markerDropServo.setPosition(0);

        while (opModeIsActive()) {

            telemetry.update();
            telemetry.addData("current Arm Position:", currentChainPos);
            telemetry.addData("Winch Position:", h.motorWinch.getCurrentPosition());
            telemetry.addData("Lift Position:", h.motorLift.getCurrentPosition());
            telemetry.addData("Gyro: ", h.MRGyro.getIntegratedZValue());
            telemetry.update();

//DRIVING
            
            
            h.driveOmniDir(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            /*
            h.motorFrontRight.setPower(gamepad1.right_stick_y);
            h.motorFrontLeft.setPower(gamepad1.left_stick_y);
            h.motorBackRight.setPower(gamepad1.right_stick_y);
            h.motorBackLeft.setPower(gamepad1.right_stick_y);
            */


            /*
            if (Throttle = true) {
                if (Math.abs(gamepad1.left_stick_y) > 0) {
                    //FORWARD BACKWARD
                    motorBackRight.setPower(Math.pow(gamepad1.left_stick_y, 3));
                    motorFrontRight.setPower(Math.pow(gamepad1.left_stick_y, 3));
                    motorFrontLeft.setPower(Math.pow(gamepad1.left_stick_y, 3));
                    motorBackLeft.setPower(Math.pow(gamepad1.left_stick_y, 3));


                } else if (Math.abs(gamepad1.right_stick_x) > 0) {
                    //TURNING
                    motorBackRight.setPower(Math.pow(gamepad1.right_stick_x, 3));
                    motorFrontRight.setPower(Math.pow(gamepad1.right_stick_x, 3));
                    motorFrontLeft.setPower(-Math.pow(gamepad1.right_stick_x, 3));
                    motorBackLeft.setPower(-Math.pow(gamepad1.right_stick_x, 3));
                } else if (Math.abs(gamepad1.left_stick_x) > 0.1)
                {
                    //MECANUM
                    motorBackRight.setPower(-gamepad1.left_stick_x/2);
                    motorFrontRight.setPower(gamepad1.left_stick_x/2);
                    motorFrontLeft.setPower(-gamepad1.left_stick_x/2);
                    motorBackLeft.setPower(gamepad1.left_stick_x/2);

                } else {
                    motorBackRight.setPower(0);
                    motorFrontRight.setPower(0);
                    motorFrontLeft.setPower(0);
                    motorBackLeft.setPower(0);

                }
            }
/*
            } else {
                motorBackRight.setPower(-gamepad1.right_stick_y);
                motorFrontRight.setPower(-gamepad1.right_stick_y);
                motorFrontLeft.setPower(gamepad1.left_stick_y);
                motorBackLeft.setPower(gamepad1.left_stick_y);
            }

            ////////WINCH
            if (gamepad1.dpad_up && motorWinch.getCurrentPosition() < 6300)
            {
                motorWinch.setPower(1);
                motorWinch.setTargetPosition(6300);
                while(gamepad1.dpad_up && motorWinch.getCurrentPosition() < 6300)
                {

                }
                motorWinch.setTargetPosition(motorWinch.getCurrentPosition());
            }
            if (gamepad1.dpad_down && motorWinch.getCurrentPosition() > 1200)
            {
                motorWinch.setPower(1);
                motorWinch.setTargetPosition(1200);
                while(gamepad1.dpad_down && motorWinch.getCurrentPosition() > 1200)
                {

                }
                motorWinch.setTargetPosition(motorWinch.getCurrentPosition());
            }
            /*
            if (!gamepad1.dpad_up && !gamepad1.dpad_down)
            {
                motorWinch.setPower(0);
            }
            */
            ////////SPINNER
            if (gamepad1.a) {
                h.motorSpinner.setPower(-1);
            }
            if (gamepad1.b) {
                h.motorSpinner.setPower(1);
            }
            if (gamepad1.x) {
                h.motorSpinner.setPower(0);
            }


////////CHAIN
            if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0) {

                //LIFT ARM

                h.motorArm.setPower(1);
                h.motorArm.setTargetPosition(4200);

                while (gamepad1.right_trigger > 0) {
                }
                currentChainPos = h.motorArm.getCurrentPosition();
            }
            if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0) {

                //DROP ARM

                h.motorArm.setPower(0.5);
                h.motorArm.setTargetPosition(50);

                while (gamepad1.left_trigger > 0) {
                }
                currentChainPos = h.motorArm.getCurrentPosition();
            }
            if ((gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0))
            {
                //motorChain.setPower(0);
                h.motorArm.setTargetPosition(currentChainPos);
            }

////////CHAIN END

////////ACTUATOR
            if (gamepad1.dpad_left && h.motorLift.getCurrentPosition() < 9400)
            {
                h.motorLift.setPower(1);
                h.motorLift.setTargetPosition(9400);
            }
            if (gamepad1.dpad_right && h.motorLift.getCurrentPosition() > 0)
            {
                h.motorLift.setPower(1);
                h.motorLift.setTargetPosition(0);
            }
            if (!gamepad1.dpad_left && !gamepad1.dpad_right)
            {
                h.motorLift.setPower(0);
            }
        }
    }
}

