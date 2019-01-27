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

    DcMotor motorBackLeft;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorFrontRight;


    DcMotor motorWinch;
    DcMotor motorChain;
    DcMotor motorLift;


    DcMotor motorStarSpinner;
    Servo markerDropServo;

    ModernRoboticsI2cColorSensor MRcolor;
    ModernRoboticsI2cGyro MRGyro;
    ModernRoboticsI2cRangeSensor MRRange;


    int currentChainPos;
    boolean Throttle = false;

    double rotation;

    public void drive(boolean forward, int distanceEncodeVal) {
        distanceEncodeVal = -(int) Math.round((distanceEncodeVal / (4 * Math.PI)) * 1120);

        motorFrontLeft.setTargetPosition(distanceEncodeVal);
        motorFrontRight.setTargetPosition(distanceEncodeVal);
        motorBackLeft.setTargetPosition(distanceEncodeVal);
        motorBackRight.setTargetPosition(distanceEncodeVal);

        if (forward) {
            motorFrontLeft.setPower(-0.3);
            motorFrontRight.setPower(0.3);
            motorBackLeft.setPower(-0.3);
            motorBackRight.setPower(0.3);
        } else {
            motorFrontLeft.setPower(0.3);
            motorFrontRight.setPower(-0.3);
            motorBackLeft.setPower(0.3);
            motorBackRight.setPower(-0.3);
        }

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*
        while((motorFrontLeft.getCurrentPosition() > distanceEncodeVal) && opModeIsActive())
        {
            telemetry.addData("Running", "...");
            telemetry.update();
        }
        */
        try {
            Thread.sleep(5000);

        } catch (Exception e) {
        }

        motorFrontLeft.setTargetPosition(0);
        motorFrontRight.setTargetPosition(0);
        motorBackLeft.setTargetPosition(0);
        motorBackRight.setTargetPosition(0);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        telemetry.clear();

    }

    @Override
    public void runOpMode() {

        try {

            h.init(hardwareMap);
            /*
            motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
            motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
            motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

            motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
            motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
            motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

            motorChain = hardwareMap.dcMotor.get("motorChain");
            motorChain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorWinch = hardwareMap.dcMotor.get("motorWinch");
            motorWinch.setDirection(DcMotorSimple.Direction.REVERSE);
            motorWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift = hardwareMap.dcMotor.get("motorLift");
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorStarSpinner = hardwareMap.dcMotor.get("motorStarSpinner");
            markerDropServo = hardwareMap.servo.get("markerDropServo");


            MRcolor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "MRcolor");
            MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
            MRRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "MRRange");
            */

        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
        }


        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorChain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorChain.setDirection(DcMotorSimple.Direction.REVERSE);

        motorWinch.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        motorWinch.setDirection(DcMotorSimple.Direction.REVERSE);

        //motorChain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorChain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

/*
        MRGyro.calibrate();
        while(MRGyro.isCalibrating())
        {
            telemetry.update();
            telemetry.addData("Gyro:", "calibrating");
        }
        telemetry.addData("Calibration", "complete");
        telemetry.update();
*/

        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        waitForStart();
        markerDropServo.setPosition(0);

        while (opModeIsActive()) {

            telemetry.update();
            telemetry.addData("current Arm Position:", currentChainPos);
            telemetry.addData("Winch Position:", motorWinch.getCurrentPosition());
            telemetry.addData("Lift Position:", motorLift.getCurrentPosition());
            telemetry.addData("Gyro: ", MRGyro.getIntegratedZValue());
            telemetry.addData("Marker Drop Servo: ", markerDropServo.getPosition());
            telemetry.update();

            //rotation = Math.pow(-gamepad1.right_stick_x, 3/2)/1.5;
            //
            /*
            rotation = -gamepad1.right_stick_x;

            motorBackRight.setPower(Math.pow(gamepad1.left_stick_y, 3 / 1) + rotation);
            motorFrontRight.setPower(Math.pow(gamepad1.left_stick_y, 3 / 1) + rotation);
            motorFrontLeft.setPower(Math.pow(gamepad1.left_stick_y, 3 / 1) - rotation);
            motorBackLeft.setPower(Math.pow(gamepad1.left_stick_y, 3 / 1) - rotation);
            */

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





                    /*
                    motorBackRight.setPower(Range.clip(gamepad1.left_stick_x, -0.4, 0.4) - Range.clip(2*(MRGyro.getIntegratedZValue())/100, 0, 0.4));
                    motorFrontRight.setPower(-Range.clip(gamepad1.left_stick_x, -0.4, 0.4) + Range.clip(2*(MRGyro.getIntegratedZValue())/100, 0, 0.4));
                    motorFrontLeft.setPower(Range.clip(gamepad1.left_stick_x, -0.4, 0.4) + Range.clip(2*(MRGyro.getIntegratedZValue())/100, 0, 0.4));
                    motorBackLeft.setPower(-Range.clip(gamepad1.left_stick_x, -0.4, 0.4) - Range.clip(2*(MRGyro.getIntegratedZValue())/100, 0, 0.4));
                    */
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
*/
           /* if (gamepad1.y) {
                Throttle = false;
                /*
                if (Throttle = true)
                {
                    Throttle = false;
                }
                else
                {
                    Throttle = true;
                }
                telemetry.clearAll();
                telemetry.addData("RESETTING DRIVE...", Thread.activeCount());
                try{
                    Thread.sleep(1000);
                }catch(Exception e){}

*/

           /* }
            if (gamepad1.x) {
                Throttle = true;
           }*/

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
                motorStarSpinner.setPower(-1);
            }
            if (gamepad1.b) {
                motorStarSpinner.setPower(1);
            }
            if (gamepad1.x) {
                motorStarSpinner.setPower(0);
            }


////////CHAIN
            if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0) {

                motorChain.setPower(1);
                motorChain.setTargetPosition(4200);
/*
                if(motorChain.getCurrentPosition() > 800 && motorChain.getCurrentPosition() < 1300)
                {
                    motorWinch.setTargetPosition(500);
                }
*/
                while (gamepad1.right_trigger > 0) {

                }
                currentChainPos = motorChain.getCurrentPosition();
            }
            if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0) {
                motorChain.setPower(0.5);
                motorChain.setTargetPosition(50);

                while (gamepad1.left_trigger > 0) {

                }
                currentChainPos = motorChain.getCurrentPosition();
            }
            if ((gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0)/* || (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0)*/) {
                //motorChain.setPower(0);
                motorChain.setTargetPosition(currentChainPos);


            }
///DEBUG ON GAMEPAD2
/*
        if (gamepad2.right_trigger > 0 && gamepad2.left_trigger == 0) {

            motorChain.setPower(0.5);
            motorChain.setTargetPosition(10000);


            while (gamepad2.right_trigger > 0) {

            }
            currentChainPos = motorChain.getCurrentPosition();
        }
        if (gamepad2.left_trigger > 0 && gamepad2.right_trigger == 0) {
            motorChain.setPower(0.5);
            motorChain.setTargetPosition(-10000);

            while (gamepad2.left_trigger > 0) {

            }
            currentChainPos = motorChain.getCurrentPosition();
        }*/
////////CHAIN END
            //landerBucket.setPosition(gamepad2.left_stick_y);
/*
            if (gamepad1.x)
            {
                landerBucket.setPosition(0.8);
            }
            if (gamepad1.y)
            {
                landerBucket.setPosition(0.2);
            }
            if(gamepad1.right_trigger > 0.5)
            {
                landerBucket.setPosition(0.5);
            }
*/

            //ACTUATOR
            if (gamepad1.dpad_left && motorLift.getCurrentPosition() < 9400) {
                motorLift.setPower(1);
                motorLift.setTargetPosition(9400);
            }
            if (gamepad1.dpad_right && motorLift.getCurrentPosition() > 0) {
                motorLift.setPower(1);
                motorLift.setTargetPosition(0);
            }
            if (!gamepad1.dpad_left && !gamepad1.dpad_right) {
                motorLift.setPower(0);
            }

            //MARKER DROP SERVO

            markerDropServo.setPosition(0.3);


        }
    }
}

