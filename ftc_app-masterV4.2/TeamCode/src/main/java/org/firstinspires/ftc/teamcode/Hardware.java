package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hardware extends LinearOpMode
{
    public DcMotor motorFrontRight;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    public DcMotor motorFrontLeft;

    public DcMotor motorArm;
    public DcMotor motorWinch;
    public DcMotor motorLift;
    public DcMotor motorSpinner;

    public Servo markerDropServo;

    ModernRoboticsI2cGyro MRGyro;
    ModernRoboticsI2cRangeSensor MRRange;

    HardwareMap map = null;
    DcMotor.RunMode initialMode = null;

    int driveTime;

    public int currentDegrees;

    Telemetry telemetry;

    @Override
    public void runOpMode()
    {
    }

    public void init(HardwareMap aMap)
    {
        motorFrontRight = aMap.dcMotor.get("motorFrontRight");
        motorBackRight = aMap.dcMotor.get("motorBackRight");
        motorBackLeft = aMap.dcMotor.get("motorBackLeft");
        motorFrontLeft = aMap.dcMotor.get("motorFrontLeft");
        motorArm = aMap.dcMotor.get("motorArm");
        motorWinch = aMap.dcMotor.get("motorWinch");
        motorLift = aMap.dcMotor.get("motorLift");
        motorSpinner = aMap.dcMotor.get("motorSpinner");

        markerDropServo = aMap.servo.get("markerDropServo");

        MRGyro = aMap.get(ModernRoboticsI2cGyro.class, "gyro");
        MRRange = aMap.get(ModernRoboticsI2cRangeSensor.class, "MRRange");



        //motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
    }


    public void drive(boolean forward, int distanceInches, double power)
    {

        int distanceEncodeVal;

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        distanceEncodeVal = -(int)Math.round((distanceInches/(4*Math.PI))*1120);
        driveTime = (distanceInches/10)*1000;



        if(forward)
        {
            motorFrontLeft.setTargetPosition(distanceEncodeVal);
            motorFrontRight.setTargetPosition(distanceEncodeVal);
            motorBackLeft.setTargetPosition(distanceEncodeVal);
            motorBackRight.setTargetPosition(distanceEncodeVal);
        }
        else
        {
            motorFrontLeft.setTargetPosition(-distanceEncodeVal);
            motorFrontRight.setTargetPosition(-distanceEncodeVal);
            motorBackLeft.setTargetPosition(-distanceEncodeVal);
            motorBackRight.setTargetPosition(-distanceEncodeVal);
        }

        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);

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

        telemetry.addData("Running", "...");
        telemetry.update();

        try{
            Thread.sleep(driveTime);

        }catch(Exception e){}

        telemetry.addData("Finished", "");
        telemetry.update();



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


    }

    public void strafe(boolean left, int distanceInches,double power)
    {
        int distanceEncodeVal;

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        distanceEncodeVal = -(int)Math.round((distanceInches/(4*Math.PI))*1120);
        driveTime = (distanceInches/10)*1000;



        if(left)
        {
            motorFrontLeft.setTargetPosition(-distanceEncodeVal);
            motorFrontRight.setTargetPosition(-distanceEncodeVal);
            motorBackLeft.setTargetPosition(distanceEncodeVal);
            motorBackRight.setTargetPosition(distanceEncodeVal);
        }
        else
        {
            motorFrontLeft.setTargetPosition(distanceEncodeVal);
            motorFrontRight.setTargetPosition(distanceEncodeVal);
            motorBackLeft.setTargetPosition(-distanceEncodeVal);
            motorBackRight.setTargetPosition(-distanceEncodeVal);
        }

        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);

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
        try{
            Thread.sleep(driveTime);

        }catch(Exception e){}


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

    }

    public void driveOmniDir(double joystickX, double joystickY, double rotation)
    {
        
        motorFrontRight.setPower(joystickY + joystickX/2 + rotation);
        motorBackRight.setPower(joystickY - joystickX/2 + rotation);
        motorFrontLeft.setPower(joystickY - joystickX/2 - rotation);
        motorBackLeft.setPower(joystickY + joystickX/2 - rotation);
    }

    public void turn(int targetDegrees, double power, double correctionPower)
    {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);

//Right is all positive
//Left is all negative
//Straight is left positive. Right negative.
        if(targetDegrees == 0)
        {
            //Tell robot to correct to straight forward

            if(MRGyro.getIntegratedZValue() > 0)
            {
                //If the gyro reads back left from zero

                motorFrontLeft.setPower(-power);
                motorBackLeft.setPower(-power);
                motorFrontRight.setPower(power);
                motorBackRight.setPower(power);

                while(MRGyro.getIntegratedZValue() > targetDegrees && opModeIsActive())
                {
                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", MRGyro.getIntegratedZValue());
                    telemetry.update();
                }

                motorFrontLeft.setPower(correctionPower);
                motorBackLeft.setPower(correctionPower);
                motorFrontRight.setPower(-correctionPower);
                motorBackRight.setPower(-correctionPower);

                while(MRGyro.getIntegratedZValue() < targetDegrees && opModeIsActive())
                {
                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", MRGyro.getIntegratedZValue());
                    telemetry.update();
                }
            }
            if(MRGyro.getIntegratedZValue() < 0)
            {
                //If the gyro reads back right from zero

                motorFrontLeft.setPower(power);
                motorBackLeft.setPower(power);
                motorFrontRight.setPower(-power);
                motorBackRight.setPower(-power);

                while(MRGyro.getIntegratedZValue() < targetDegrees && opModeIsActive())
                {
                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", MRGyro.getIntegratedZValue());
                    telemetry.update();
                }
                motorFrontLeft.setPower(-correctionPower);
                motorBackLeft.setPower(-correctionPower);
                motorFrontRight.setPower(correctionPower);
                motorBackRight.setPower(correctionPower);

                while(MRGyro.getIntegratedZValue() > targetDegrees && opModeIsActive())
                {
                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", MRGyro.getIntegratedZValue());
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
            motorFrontLeft.setPower(-power);
            motorBackLeft.setPower(-power);
            motorFrontRight.setPower(power);
            motorBackRight.setPower(power);

            while(MRGyro.getIntegratedZValue() > targetDegrees && opModeIsActive())
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", MRGyro.getIntegratedZValue());
                telemetry.update();
            }

            motorFrontLeft.setPower(correctionPower);
            motorBackLeft.setPower(correctionPower);
            motorFrontRight.setPower(-correctionPower);
            motorBackRight.setPower(-correctionPower);

            while(MRGyro.getIntegratedZValue() < targetDegrees && opModeIsActive())
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", MRGyro.getIntegratedZValue());
                telemetry.update();
            }
        }
        else
        {
            //LEFT
            motorFrontLeft.setPower(-power);
            motorBackLeft.setPower(-power);
            motorFrontRight.setPower(power);
            motorBackRight.setPower(power);

            while(MRGyro.getIntegratedZValue() < targetDegrees && opModeIsActive())
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", MRGyro.getIntegratedZValue());
                telemetry.update();
            }
            motorFrontLeft.setPower(correctionPower);
            motorBackLeft.setPower(correctionPower);
            motorFrontRight.setPower(-correctionPower);
            motorBackRight.setPower(-correctionPower);

            while(MRGyro.getIntegratedZValue() > targetDegrees && opModeIsActive())
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", MRGyro.getIntegratedZValue());
                telemetry.update();
            }
        }
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

}
