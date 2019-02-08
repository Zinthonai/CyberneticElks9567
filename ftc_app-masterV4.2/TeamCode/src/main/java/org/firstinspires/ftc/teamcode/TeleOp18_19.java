package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

    OpMode opmode;



    int currentChainPos;

    @Override
    public void runOpMode()
    {

        Hardware h = new Hardware();

        try {
            h.init(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            e.printStackTrace();
        }

        telemetry.addData("Initialization ", "complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Current Arm Position:", h.motorArm.getCurrentPosition());
            telemetry.addData("Winch Position:", h.motorWinch.getCurrentPosition());
            telemetry.addData("Lift Position:", h.motorLift.getCurrentPosition());
            //telemetry.addData("Gyro: ", h.MRGyro.getIntegratedZValue());
            telemetry.addData("bucketFlapServo", h.bucketFlapServo.getPosition());
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
            if (gamepad1.dpad_up && h.motorWinch.getCurrentPosition() < 3250)
            {
                h.motorWinch.setPower(1);
                h.motorWinch.setTargetPosition(3250);
                while(gamepad1.dpad_up && h.motorWinch.getCurrentPosition() < 3250 && opModeIsActive())
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
                h.motorWinch.setTargetPosition(h.motorWinch.getCurrentPosition());
            }

////////SPINNER
            if (gamepad1.a) {
                h.motorSpinner.setPower(-0.5);
            }
            if (gamepad1.b) {
                h.motorSpinner.setPower(0.3);
            }
            if (gamepad1.x) {
                h.motorSpinner.setPower(0);
            }

////////ARM
            if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0) {

                //LIFT ARM

                h.motorArm.setPower(1);
                h.motorArm.setTargetPosition(10500);

                while (gamepad1.right_trigger > 0 && opModeIsActive()) {
                }
                currentChainPos = h.motorArm.getCurrentPosition();
            }
            if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0) {

                //DROP ARM

                h.motorArm.setPower(0.8);
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
            if (gamepad1.dpad_left && h.motorLift.getCurrentPosition() < 10100)
            {
                h.motorLift.setPower(1);
                h.motorLift.setTargetPosition(10100);
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
///////////FLAP SERVO
            if(gamepad1.y)
            {
                h.bucketFlapServo.setPosition(0.45);
            }
            else
            {
                h.bucketFlapServo.setPosition(0);
            }



        }
    }
}

