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

@TeleOp (name = "TeleOp Rover Ruckus", group = "TeleOp")
public class TeleOp18_19 extends LinearOpMode
{



    int currentChainPos;

    @Override
    public void runOpMode()
    {

        Hardware h = new Hardware();

        try {
            h.init(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            e.printStackTrace();
        }


        h.MRGyro.calibrate();
        while(h.MRGyro.isCalibrating() && opModeIsActive())
        {
            telemetry.update();
            telemetry.addData("Gyro:", "calibrating");
        }
        telemetry.addData("Calibration", "complete");
        telemetry.update();

        waitForStart();
        h.markerDropServo.setPosition(0);

        while (opModeIsActive())
        {
            telemetry.addData("current Arm Position:", h.motorArm.getCurrentPosition());
            telemetry.addData("Winch Position:", h.motorWinch.getCurrentPosition());
            telemetry.addData("Lift Position:", h.motorLift.getCurrentPosition());
            telemetry.addData("Gyro: ", h.MRGyro.getIntegratedZValue());
            telemetry.update();

////////DRIVING

            h.driveOmniDir(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            //TANK DRIVE TEST

/*
            h.motorFrontRight.setPower(gamepad1.right_stick_y);
            h.motorFrontLeft.setPower(gamepad1.left_stick_y);
            h.motorBackRight.setPower(gamepad1.right_stick_y);
            h.motorBackLeft.setPower(gamepad1.right_stick_y);
*/

////////WINCH
            if (gamepad1.dpad_up && h.motorWinch.getCurrentPosition() < 6300)
            {
                h.motorWinch.setPower(1);
                h.motorWinch.setTargetPosition(6300);
                while(gamepad1.dpad_up && h.motorWinch.getCurrentPosition() < 6300 && opModeIsActive())
                {

                }
                h.motorWinch.setTargetPosition(h.motorWinch.getCurrentPosition());
            }
            if (gamepad1.dpad_down && h.motorWinch.getCurrentPosition() > 0)
            {
                h.motorWinch.setPower(1);
                h.motorWinch.setTargetPosition(0);
                while(gamepad1.dpad_down && h.motorWinch.getCurrentPosition() > 0 && opModeIsActive())
                {

                }
                h.motorWinch.setTargetPosition(h.motorWinch.getCurrentPosition());
            }
            if (!gamepad1.dpad_up && !gamepad1.dpad_down)
            {
                h.motorWinch.setPower(0);
            }

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

////////ARM
            if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0) {

                //LIFT ARM

                h.motorArm.setPower(0.5);
                h.motorArm.setTargetPosition(4200);

                while (gamepad1.right_trigger > 0 && opModeIsActive()) {
                }
                currentChainPos = h.motorArm.getCurrentPosition();
            }
            if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0) {

                //DROP ARM

                h.motorArm.setPower(0.3);
                h.motorArm.setTargetPosition(0);

                while (gamepad1.left_trigger > 0 && opModeIsActive()) {
                }
                currentChainPos = h.motorArm.getCurrentPosition();
            }
            if ((gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0))
            {
                h.motorArm.setPower(0);
                h.motorArm.setTargetPosition(currentChainPos);
            }

////////ARM END

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

