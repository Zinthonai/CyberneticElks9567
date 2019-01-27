package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Created by Grace on 1/27/2018.
 */

public class Main extends LinearOpMode
{
    public DcMotor motorBackLeft;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorFrontRight;

    public DcMotor pinion1;
    public DcMotor pinion2;

    public DcMotor winch;
    public DcMotor xrailLift;

    public Servo clawLeft;
    public Servo clawRight;
    public Servo JewelArm;
    public Servo xClaw;

    public ModernRoboticsI2cColorSensor MRcolor;
    public ModernRoboticsI2cGyro MRGyro;

    public double leftPower;
    public double rightPower;

    public double rightRotation;
    public double rotation;
    @Override
    public void runOpMode() throws InterruptedException {}

    public void initializeRobot()
    {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        pinion1 = hardwareMap.dcMotor.get("pinion1");
        pinion2 = hardwareMap.dcMotor.get("pinion2");
        winch = hardwareMap.dcMotor.get("winch");
        xrailLift = hardwareMap.dcMotor.get("xrailLift");
        clawLeft = hardwareMap.servo.get("clawLeft");
        clawRight = hardwareMap.servo.get("clawRight");
        JewelArm = hardwareMap.servo.get("JewelArm");
        MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        MRcolor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "MRcolor");

        //REVcolor = hardwareMap.colorSensor.get("REVcolor");

        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        pinion2.setDirection(DcMotorSimple.Direction.REVERSE);
        pinion1.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Zero power set
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pinion1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pinion2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        xrailLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MRGyro.calibrate();
        while(MRGyro.isCalibrating())
        {
            telemetry.update();
            telemetry.addData("Gyro:", "calibrating");
        }
        telemetry.addData("Calibration", "complete");
        telemetry.update();
    }
    public void drive (boolean forward, int distanceEncodeVal)
    {
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

    public void clawOpen()
    {
        clawLeft.setPosition(0);
        clawRight.setPosition(0.25);
    }
    public void clawClose()
    {
        clawLeft.setPosition(0.3);
        clawRight.setPosition(0);
    }

    public void blockRack(int blockArmPos)
    {
        pinion1.setPower(1);
        pinion2.setPower(1);
        switch (blockArmPos)
        {
            case 0:
                pinion1.setTargetPosition(0);
                pinion2.setTargetPosition(0);
                pinion1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pinion2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;

            case 1:
                pinion1.setTargetPosition(1100);
                pinion2.setTargetPosition(0);
                pinion1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pinion2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;

            case 2:
                pinion1.setTargetPosition(1100);
                pinion2.setTargetPosition(1250);
                pinion1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pinion2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;

            case 3:
                pinion1.setTargetPosition(1100);
                pinion2.setTargetPosition(2800);
                pinion1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pinion2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
        }
    }

    public void turn(int targetDegrees)
    {
        motorFrontLeft.setPower(0.5);
        motorBackLeft.setPower(0.5);
        motorFrontRight.setPower(0.5);
        motorBackRight.setPower(0.5);

//Right is all positive
//Left is all negative
//Straight is left positive. Right negative.
        if(targetDegrees == 0)
        {
            if(MRGyro.getIntegratedZValue() > 0)
            {
                //Left of zero
                motorFrontLeft.setTargetPosition(10000);
                motorBackLeft.setTargetPosition(10000);
                motorFrontRight.setTargetPosition(10000);
                motorBackRight.setTargetPosition(10000);

                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while(MRGyro.getIntegratedZValue() > targetDegrees)
                {
                    telemetry.addData("turning right", "yes");
                    telemetry.addData("MRgyro", MRGyro.getIntegratedZValue());
                    telemetry.update();

                }
                motorFrontLeft.setPower(0.1);
                motorBackLeft.setPower(0.1);
                motorFrontRight.setPower(0.1);
                motorBackRight.setPower(0.1);

                motorFrontLeft.setTargetPosition(-10000);
                motorBackLeft.setTargetPosition(-10000);
                motorFrontRight.setTargetPosition(-10000);
                motorBackRight.setTargetPosition(-10000);

                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while(MRGyro.getIntegratedZValue() < targetDegrees)
                {
                    telemetry.addData("correcting", "yeah");
                    telemetry.update();
                }

            }
            if(MRGyro.getIntegratedZValue() < 0)
            {
                //Right of zero

                motorFrontLeft.setTargetPosition(-10000);
                motorBackLeft.setTargetPosition(-10000);
                motorFrontRight.setTargetPosition(-10000);
                motorBackRight.setTargetPosition(-10000);

                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(MRGyro.getIntegratedZValue() < targetDegrees)
                {
                    telemetry.addData("turning left", "yes");
                    telemetry.addData("MRgyro", MRGyro.getIntegratedZValue());
                    telemetry.update();
                }
                motorFrontLeft.setPower(0.1);
                motorBackLeft.setPower(0.1);
                motorFrontRight.setPower(0.1);
                motorBackRight.setPower(0.1);

                motorFrontLeft.setTargetPosition(10000);
                motorBackLeft.setTargetPosition(10000);
                motorFrontRight.setTargetPosition(10000);
                motorBackRight.setTargetPosition(10000);

                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while(MRGyro.getIntegratedZValue() > targetDegrees)
                {
                    telemetry.addData("correcting to the right", "yeah");
                    telemetry.update();
                }
            }
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackRight.setPower(0);
        }

        if(targetDegrees < 0)
        {

            //RIGHT
            motorFrontLeft.setTargetPosition(10000);
            motorBackLeft.setTargetPosition(10000);
            motorFrontRight.setTargetPosition(10000);
            motorBackRight.setTargetPosition(10000);

            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(MRGyro.getIntegratedZValue() > targetDegrees)
            {
                telemetry.addData("turning right", "yes");
                telemetry.addData("MRgyro", MRGyro.getIntegratedZValue());
                telemetry.update();

            }
            motorFrontLeft.setPower(0.1);
            motorBackLeft.setPower(0.1);
            motorFrontRight.setPower(0.1);
            motorBackRight.setPower(0.1);

            motorFrontLeft.setTargetPosition(-10000);
            motorBackLeft.setTargetPosition(-10000);
            motorFrontRight.setTargetPosition(-10000);
            motorBackRight.setTargetPosition(-10000);

            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(MRGyro.getIntegratedZValue() < targetDegrees)
            {
                telemetry.addData("correcting", "yeah");
                telemetry.update();
            }

        }
        else
        {
            //LEFT

            motorFrontLeft.setTargetPosition(-10000);
            motorBackLeft.setTargetPosition(-10000);
            motorFrontRight.setTargetPosition(-10000);
            motorBackRight.setTargetPosition(-10000);

            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(MRGyro.getIntegratedZValue() < targetDegrees)
            {
                telemetry.addData("turning left", "yes");
                telemetry.addData("MRgyro", MRGyro.getIntegratedZValue());
                telemetry.update();
            }
            motorFrontLeft.setPower(0.1);
            motorBackLeft.setPower(0.1);
            motorFrontRight.setPower(0.1);
            motorBackRight.setPower(0.1);

            motorFrontLeft.setTargetPosition(10000);
            motorBackLeft.setTargetPosition(10000);
            motorFrontRight.setTargetPosition(10000);
            motorBackRight.setTargetPosition(10000);

            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(MRGyro.getIntegratedZValue() > targetDegrees)
            {
                telemetry.addData("correcting to the right", "yeah");
                telemetry.update();
            }
        }
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }
}
