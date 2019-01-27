/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */



package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "Rover Ruckus Autonomous Depot", group = "Linear Opmode_Auto")

public class RoverRuckusAutonomousDepot extends LinearOpMode
{
    private String Date;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorBackLeft;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontRight;


    DcMotor motorWinch;
    DcMotor motorChain;
    DcMotor motorLift;
    DcMotor motorStarSpinner;

    Servo markerDropServo;

    ModernRoboticsI2cGyro MRGyro;

    ModernRoboticsI2cColorSensor MRcolor;
    ColorSensor REVcolor;

    public int currentDegrees;

    long start = System.currentTimeMillis();

    int driveTime;

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

        telemetry.clear();

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

        telemetry.clear();
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
                    telemetry.addData("turning right", "yes");
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
                    telemetry.addData("correcting", "yeah");
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
                telemetry.addData("turning right", "yes");
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
                telemetry.addData("correcting", "yeah");
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

    ////////////////START////////////////
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.update();
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorChain = hardwareMap.dcMotor.get("motorChain");
        motorChain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift = hardwareMap.dcMotor.get("motorLift");
        motorWinch = hardwareMap.dcMotor.get("motorWinch");
        motorWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorStarSpinner = hardwareMap.dcMotor.get("motorStarSpinner");

////////////////SERVOS/////////////////


        markerDropServo = hardwareMap.servo.get("markerDropServo");

////////////////SENSORS////////////////
        MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        MRcolor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "MRcolor");

        //REVcolor = hardwareMap.colorSensor.get("REVcolor");

        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Zero power set
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorChain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorWinch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Calibrate gyro

        MRGyro.calibrate();
        while(MRGyro.isCalibrating())
        {
            telemetry.update();
            telemetry.addData("Gyro:", "calibrating");
        }
        telemetry.addData("Calibration", "complete");
        telemetry.update();

        waitForStart();
////////////////AUTONOMOUS////////////////

        motorLift.setTargetPosition(9500);
        motorLift.setPower(1);

        Thread.sleep(4500);

        motorLift.setTargetPosition(0);

        turn(45);

        drive(true, 2, 1);

        turn(0);

        //motorChain.setPower(0.3);
        //motorChain.setTargetPosition(600);

        //Thread.sleep(1000);

        drive(true, 42, 1);

        //motorChain.setTargetPosition(300);

        turn(180);

        markerDropServo.setPosition(0);
        Thread.sleep(2000);
        markerDropServo.setPosition(0.3);

        turn(135);

        drive(true, 70, 1);

        while(opModeIsActive())
        {
            telemetry.addData("motorBackLeft", motorBackLeft.getCurrentPosition());
            telemetry.addData("motorBackRight", motorBackRight.getCurrentPosition());
            telemetry.addData("motorFrontLeft", motorFrontLeft.getCurrentPosition());
            telemetry.addData("motorFrontRight", motorFrontRight.getCurrentPosition());

            telemetry.addData("gyro:", MRGyro.getHeading());
            telemetry.addData("integated z:", MRGyro.getIntegratedZValue());
            telemetry.addData("current degrees:", currentDegrees);

            telemetry.update();
        }
    }
}