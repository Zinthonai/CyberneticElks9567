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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.Main;


@Autonomous (name = "Rover Ruckus Autonomous Crater", group = "Linear Opmode_Auto")

public class RoverRuckusAutonomous extends LinearOpMode
{
    Hardware h = new Hardware(DcMotor.RunMode.RUN_TO_POSITION);

    private String Date;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorBackLeft;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontRight;


    DcMotor motorWinch;
    DcMotor motorArm;
    DcMotor motorLift;
    DcMotor motorSpinner;

    Servo markerDropServo;

    ModernRoboticsI2cGyro MRGyro;

    ModernRoboticsI2cColorSensor MRcolor;
    ColorSensor REVcolor;

    public int currentDegrees;

    long start = System.currentTimeMillis();

    int driveTime;

    public void drive(boolean forward, int distanceInches,double power)
    {

        int distanceEncodeVal;

        h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        distanceEncodeVal = -(int)Math.round((distanceInches/(4*Math.PI))*1120);
        driveTime = (distanceInches/10)*1000;



        if(forward)
        {
            h.motorFrontLeft.setTargetPosition(distanceEncodeVal);
            h.motorFrontRight.setTargetPosition(distanceEncodeVal);
            h.motorBackLeft.setTargetPosition(distanceEncodeVal);
            h.motorBackRight.setTargetPosition(distanceEncodeVal);

            h.motorFrontLeft.setPower(-power);
            h.motorFrontRight.setPower(power);
            h.motorBackLeft.setPower(-power);
            h.motorBackRight.setPower(power);
        }
        else
        {
            h.motorFrontLeft.setTargetPosition(-distanceEncodeVal);
            h.motorFrontRight.setTargetPosition(-distanceEncodeVal);
            h.motorBackLeft.setTargetPosition(-distanceEncodeVal);
            h.motorBackRight.setTargetPosition(-distanceEncodeVal);

            h.motorFrontLeft.setPower(power);
            h.motorFrontRight.setPower(-power);
            h.motorBackLeft.setPower(power);
            h.motorBackRight.setPower(-power);
        }

        h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*
        while((h.motorFrontLeft.getCurrentPosition() > distanceEncodeVal) && opModeIsActive())
        {
            telemetry.addData("Running", "...");
            telemetry.update();
        }
        */
        try{
            Thread.sleep(driveTime);

        }catch(Exception e){}




        h.motorFrontLeft.setTargetPosition(0);
        h.motorFrontRight.setTargetPosition(0);
        h.motorBackLeft.setTargetPosition(0);
        h.motorBackRight.setTargetPosition(0);

        h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        h.motorFrontLeft.setPower(0);
        h.motorFrontRight.setPower(0);
        h.motorBackLeft.setPower(0);
        h.motorBackRight.setPower(0);

        telemetry.clear();

    }

    public void strafe(boolean left, int distanceInches,  double power)
    {
        int distanceEncodeVal;

        h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        distanceEncodeVal = -(int)Math.round((distanceInches/(4*Math.PI))*1120);
        driveTime = (distanceInches/10)*1000;



        if(left)
        {
            h.motorFrontLeft.setTargetPosition(-distanceEncodeVal);
            h.motorFrontRight.setTargetPosition(-distanceEncodeVal);
            h.motorBackLeft.setTargetPosition(distanceEncodeVal);
            h.motorBackRight.setTargetPosition(distanceEncodeVal);

            h.motorFrontLeft.setPower(power);
            h.motorFrontRight.setPower(power);
            h.motorBackLeft.setPower(-power);
            h.motorBackRight.setPower(-power);
        }
        else
        {
            h.motorFrontLeft.setTargetPosition(distanceEncodeVal);
            h.motorFrontRight.setTargetPosition(distanceEncodeVal);
            h.motorBackLeft.setTargetPosition(-distanceEncodeVal);
            h.motorBackRight.setTargetPosition(-distanceEncodeVal);

            h.motorFrontLeft.setPower(-power);
            h.motorFrontRight.setPower(-power);
            h.motorBackLeft.setPower(power);
            h.motorBackRight.setPower(power);
        }

        h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);





        /*
        while((h.motorFrontLeft.getCurrentPosition() > distanceEncodeVal) && opModeIsActive())
        {
            telemetry.addData("Running", "...");
            telemetry.update();
        }
        */
        try{
            Thread.sleep(driveTime);

        }catch(Exception e){}




        h.motorFrontLeft.setTargetPosition(0);
        h.motorFrontRight.setTargetPosition(0);
        h.motorBackLeft.setTargetPosition(0);
        h.motorBackRight.setTargetPosition(0);

        h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        h.motorFrontLeft.setPower(0);
        h.motorFrontRight.setPower(0);
        h.motorBackLeft.setPower(0);
        h.motorBackRight.setPower(0);

        telemetry.clear();
}
    public void turn(int targetDegrees)
    {
        h.motorFrontLeft.setPower(0.5);
        h.motorBackLeft.setPower(0.5);
        h.motorFrontRight.setPower(0.5);
        h.motorBackRight.setPower(0.5);

//Right is all positive
//Left is all negative
//Straight is left positive. Right negative.
        if(targetDegrees == 0)
        {
            if(MRGyro.getIntegratedZValue() > 0)
            {
                //Left of zero
                h.motorFrontLeft.setTargetPosition(-10000);
                h.motorBackLeft.setTargetPosition(-10000);
                h.motorFrontRight.setTargetPosition(10000);
                h.motorBackRight.setTargetPosition(10000);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while(MRGyro.getIntegratedZValue() > targetDegrees && opModeIsActive())
                {
                    telemetry.addData("turning right", "yes");
                    telemetry.addData("MRgyro", MRGyro.getIntegratedZValue());
                    telemetry.update();

                }
                h.motorFrontLeft.setPower(0.1);
                h.motorBackLeft.setPower(0.1);
                h.motorFrontRight.setPower(0.1);
                h.motorBackRight.setPower(0.1);

                h.motorFrontLeft.setTargetPosition(10000);
                h.motorBackLeft.setTargetPosition(10000);
                h.motorFrontRight.setTargetPosition(-10000);
                h.motorBackRight.setTargetPosition(-10000);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while(MRGyro.getIntegratedZValue() < targetDegrees && opModeIsActive())
                {
                    telemetry.addData("correcting", "yeah");
                    telemetry.update();
                }

            }
            if(MRGyro.getIntegratedZValue() < 0)
            {
                //Right of zero

                h.motorFrontLeft.setTargetPosition(10000);
                h.motorBackLeft.setTargetPosition(10000);
                h.motorFrontRight.setTargetPosition(-10000);
                h.motorBackRight.setTargetPosition(-10000);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(MRGyro.getIntegratedZValue() < targetDegrees && opModeIsActive())
                {
                    telemetry.addData("turning left", "yes");
                    telemetry.addData("MRgyro", MRGyro.getIntegratedZValue());
                    telemetry.update();
                }
                h.motorFrontLeft.setPower(0.1);
                h.motorBackLeft.setPower(0.1);
                h.motorFrontRight.setPower(0.1);
                h.motorBackRight.setPower(0.1);

                h.motorFrontLeft.setTargetPosition(-10000);
                h.motorBackLeft.setTargetPosition(-10000);
                h.motorFrontRight.setTargetPosition(10000);
                h.motorBackRight.setTargetPosition(10000);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while(MRGyro.getIntegratedZValue() > targetDegrees && opModeIsActive())
                {
                    telemetry.addData("correcting to the right", "yeah");
                    telemetry.update();
                }
            }
            h.motorFrontLeft.setPower(0);
            h.motorBackLeft.setPower(0);
            h.motorFrontRight.setPower(0);
            h.motorBackRight.setPower(0);
        }

        if(targetDegrees < 0)
        {

            //RIGHT
            h.motorFrontLeft.setTargetPosition(-10000);
            h.motorBackLeft.setTargetPosition(-10000);
            h.motorFrontRight.setTargetPosition(10000);
            h.motorBackRight.setTargetPosition(10000);

            h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(MRGyro.getIntegratedZValue() > targetDegrees && opModeIsActive())
            {
                telemetry.addData("turning right", "yes");
                telemetry.addData("MRgyro", MRGyro.getIntegratedZValue());
                telemetry.update();

            }
            h.motorFrontLeft.setPower(0.1);
            h.motorBackLeft.setPower(0.1);
            h.motorFrontRight.setPower(0.1);
            h.motorBackRight.setPower(0.1);

            h.motorFrontLeft.setTargetPosition(10000);
            h.motorBackLeft.setTargetPosition(10000);
            h.motorFrontRight.setTargetPosition(-10000);
            h.motorBackRight.setTargetPosition(-10000);

            h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(MRGyro.getIntegratedZValue() < targetDegrees && opModeIsActive())
            {
                telemetry.addData("correcting", "yeah");
                telemetry.update();
            }

        }
        else
        {
            //LEFT

            h.motorFrontLeft.setTargetPosition(10000);
            h.motorBackLeft.setTargetPosition(10000);
            h.motorFrontRight.setTargetPosition(-10000);
            h.motorBackRight.setTargetPosition(-10000);

            h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(MRGyro.getIntegratedZValue() < targetDegrees && opModeIsActive())
            {
                telemetry.addData("turning left", "yes");
                telemetry.addData("MRgyro", MRGyro.getIntegratedZValue());
                telemetry.update();
            }
            h.motorFrontLeft.setPower(0.1);
            h.motorBackLeft.setPower(0.1);
            h.motorFrontRight.setPower(0.1);
            h.motorBackRight.setPower(0.1);

            h.motorFrontLeft.setTargetPosition(-10000);
            h.motorBackLeft.setTargetPosition(-10000);
            h.motorFrontRight.setTargetPosition(10000);
            h.motorBackRight.setTargetPosition(10000);

            h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(MRGyro.getIntegratedZValue() > targetDegrees && opModeIsActive())
            {
                telemetry.addData("correcting to the right", "yeah");
                telemetry.update();
            }
        }
        h.motorFrontLeft.setPower(0);
        h.motorBackLeft.setPower(0);
        h.motorFrontRight.setPower(0);
        h.motorBackRight.setPower(0);
    }

    ////////////////START////////////////
    @Override
    public void runOpMode() throws InterruptedException
    {
        h.init(hardwareMap);
        telemetry.update();
        h.motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        h.motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        h.motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        h.motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        h.motorArm = hardwareMap.dcMotor.get("motorArm");
        h.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorLift = hardwareMap.dcMotor.get("motorLift");
        h.motorWinch = hardwareMap.dcMotor.get("motorWinch");
        h.motorWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        h.motorSpinner = hardwareMap.dcMotor.get("motorSpinner");

////////////////SERVOS/////////////////

        markerDropServo = hardwareMap.servo.get("markerDropServo");

////////////////SENSORS////////////////
        MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        MRcolor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "MRcolor");

        //REVcolor = hardwareMap.colorSensor.get("REVcolor");

        h.motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        h.motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        h.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        h.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        h.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Zero power set
        h.motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        h.motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        h.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        h.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        h.motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        h.motorWinch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        h.motorLift.setTargetPosition(9500);
        h.motorLift.setPower(1);

        Thread.sleep(4000);

        h.motorLift.setTargetPosition(0);
/*
        strafe(false,2,0.5);

        drive(true,15,1);

        strafe(true,30,.5);

        turn(45);
*/

       turn(45);

        drive(true, 2, 1);

        turn(0);

        drive(true, 15, 1);

        turn(90);

        drive(true, 35, 1);

        turn(315);

        drive(false, 46, 1);

        markerDropServo.setPosition(0);
        Thread.sleep(2000);
        markerDropServo.setPosition(0.3);

        turn(325);

        drive(true, 75, 1);

        //h.motorArm.setTargetPosition(0);


/*
        h.motorArm.setPower(0.3);
        h.motorArm.setTargetPosition(600);
        Thread.sleep(1000);
        h.motorSpinner.setPower(1); //Get Block

        drive(true, 15, 1);

        h.motorSpinner.setPower(0);

        h.motorArm.setPower(0.4);
        h.motorArm.setTargetPosition(300);

        drive(false, 5, 1);

        turn(270);

        drive(false, 22, 1);

        turn(320);



        drive(false, 50, 1);

        landerBucket.setPosition(0.8);
        Thread.sleep(2000);
        landerBucket.setPosition(0.2);

        turn(330);

        drive(true, 75, 1);

        h.motorArm.setPower(0);
*/


        while(opModeIsActive())
        {
            telemetry.addData("motorBackLeft", h.motorBackLeft.getCurrentPosition());
            telemetry.addData("motorBackRight", h.motorBackRight.getCurrentPosition());
            telemetry.addData("motorFrontLeft", h.motorFrontLeft.getCurrentPosition());
            telemetry.addData("motorFrontRight", h.motorFrontRight.getCurrentPosition());

            telemetry.addData("gyro:", MRGyro.getHeading());
            telemetry.addData("integated z:", MRGyro.getIntegratedZValue());
            telemetry.addData("current degrees:", currentDegrees);

            telemetry.update();
        }
    }
}