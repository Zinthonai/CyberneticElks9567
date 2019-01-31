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
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorFrontLeft = null;

    public DcMotor motorArm = null;
    public DcMotor motorWinch = null;
    public DcMotor motorLift = null;
    public DcMotor motorSpinner = null;

    public Servo markerDropServo = null;

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
        motorFrontLeft = aMap.dcMotor.get("motorBackLeft");
        motorFrontLeft = aMap.dcMotor.get("motorFrontLeft");
        motorArm = aMap.dcMotor.get("motorArm");
        motorWinch = aMap.dcMotor.get("motorWinch");
        motorLift = aMap.dcMotor.get("motorLift");
        motorSpinner = aMap.dcMotor.get("motorSpinner");

        markerDropServo = aMap.servo.get("markerDropServo");

        MRGyro = aMap.get(ModernRoboticsI2cGyro.class, "gyro");
        MRRange = aMap.get(ModernRoboticsI2cRangeSensor.class, "MRRange");

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

            motorFrontLeft.setPower(-power);
            motorFrontRight.setPower(power);
            motorBackLeft.setPower(-power);
            motorBackRight.setPower(power);
        }
        else
        {
            motorFrontLeft.setTargetPosition(-distanceEncodeVal);
            motorFrontRight.setTargetPosition(-distanceEncodeVal);
            motorBackLeft.setTargetPosition(-distanceEncodeVal);
            motorBackRight.setTargetPosition(-distanceEncodeVal);

            motorFrontLeft.setPower(power);
            motorFrontRight.setPower(-power);
            motorBackLeft.setPower(power);
            motorBackRight.setPower(-power);
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

            motorFrontLeft.setPower(power);
            motorFrontRight.setPower(power);
            motorBackLeft.setPower(power);
            motorBackRight.setPower(power);
        }
        else
        {
            motorFrontLeft.setTargetPosition(distanceEncodeVal);
            motorFrontRight.setTargetPosition(distanceEncodeVal);
            motorBackLeft.setTargetPosition(-distanceEncodeVal);
            motorBackRight.setTargetPosition(-distanceEncodeVal);

            motorFrontLeft.setPower(power);
            motorFrontRight.setPower(power);
            motorBackLeft.setPower(power);
            motorBackRight.setPower(power);
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
        
        motorFrontRight.setPower(joystickY + joystickX - rotation);
        motorBackRight.setPower(joystickY + joystickX - rotation);
        motorFrontLeft.setPower(joystickY - joystickX + rotation);
        motorBackLeft.setPower(joystickY - joystickX + rotation);
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
                motorFrontLeft.setTargetPosition(-10000);
                motorBackLeft.setTargetPosition(-10000);
                motorFrontRight.setTargetPosition(10000);
                motorBackRight.setTargetPosition(10000);

                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while(MRGyro.getIntegratedZValue() > targetDegrees && opModeIsActive())
                {
                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("MRgyro", MRGyro.getIntegratedZValue());
                    telemetry.update();

                }
                motorFrontLeft.setPower(0.1);
                motorBackLeft.setPower(0.1);
                motorFrontRight.setPower(0.1);
                motorBackRight.setPower(0.1);

                motorFrontLeft.setTargetPosition(10000);
                motorBackLeft.setTargetPosition(10000);
                motorFrontRight.setTargetPosition(-10000);
                motorBackRight.setTargetPosition(-10000);

                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while(MRGyro.getIntegratedZValue() < targetDegrees && opModeIsActive())
                {
                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", MRGyro.getIntegratedZValue());
                    telemetry.update();
                }

            }
            if(MRGyro.getIntegratedZValue() < 0)
            {
                //Right of zero

                motorFrontLeft.setTargetPosition(10000);
                motorBackLeft.setTargetPosition(10000);
                motorFrontRight.setTargetPosition(-10000);
                motorBackRight.setTargetPosition(-10000);

                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(MRGyro.getIntegratedZValue() < targetDegrees && opModeIsActive())
                {
                    telemetry.addData("Target Value: ", targetDegreees);
                    telemetry.addData("MRgyro", MRGyro.getIntegratedZValue());
                    telemetry.update();
                }
                motorFrontLeft.setPower(0.1);
                motorBackLeft.setPower(0.1);
                motorFrontRight.setPower(0.1);
                motorBackRight.setPower(0.1);

                motorFrontLeft.setTargetPosition(-10000);
                motorBackLeft.setTargetPosition(-10000);
                motorFrontRight.setTargetPosition(10000);
                motorBackRight.setTargetPosition(10000);

                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
            motorFrontLeft.setTargetPosition(-10000);
            motorBackLeft.setTargetPosition(-10000);
            motorFrontRight.setTargetPosition(10000);
            motorBackRight.setTargetPosition(10000);

            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(MRGyro.getIntegratedZValue() > targetDegrees && opModeIsActive())
            {
                telemetry.addData("Target: ", targetDegrees);
                telemetry.addData("Current Value: ", MRGyro.getIntegratedZValue());
                telemetry.update();

            }
            motorFrontLeft.setPower(0.1);
            motorBackLeft.setPower(0.1);
            motorFrontRight.setPower(0.1);
            motorBackRight.setPower(0.1);

            motorFrontLeft.setTargetPosition(10000);
            motorBackLeft.setTargetPosition(10000);
            motorFrontRight.setTargetPosition(-10000);
            motorBackRight.setTargetPosition(-10000);

            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(MRGyro.getIntegratedZValue() < targetDegrees && opModeIsActive())
            {
                telemetry.addData("Heading", MRGyro.getIntegratedZValue());
                telemetry.update();
            }

        }
        else
        {
            //LEFT

            motorFrontLeft.setTargetPosition(10000);
            motorBackLeft.setTargetPosition(10000);
            motorFrontRight.setTargetPosition(-10000);
            motorBackRight.setTargetPosition(-10000);

            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(MRGyro.getIntegratedZValue() < targetDegrees && opModeIsActive())
            {
                telemetry.addData("turning left", "yes");
                telemetry.addData("MRgyro", MRGyro.getIntegratedZValue());
                telemetry.update();
            }
            motorFrontLeft.setPower(0.1);
            motorBackLeft.setPower(0.1);
            motorFrontRight.setPower(0.1);
            motorBackRight.setPower(0.1);

            motorFrontLeft.setTargetPosition(-10000);
            motorBackLeft.setTargetPosition(-10000);
            motorFrontRight.setTargetPosition(10000);
            motorBackRight.setTargetPosition(10000);

            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(MRGyro.getIntegratedZValue() > targetDegrees && opModeIsActive())
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

    public Hardware(DcMotor.RunMode enteredMode)
    {


    }

}
