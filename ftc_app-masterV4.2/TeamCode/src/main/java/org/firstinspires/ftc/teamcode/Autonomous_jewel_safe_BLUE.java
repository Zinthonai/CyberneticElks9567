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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "Autonomous_jewel_safe_BLUE", group = "Linear Opmode_Auto")
@Disabled
public class Autonomous_jewel_safe_BLUE extends LinearOpMode
{
    private String Date;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorBackLeft;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontRight;

    private DcMotor pinion1;
    private DcMotor pinion2;

    private DcMotor winch;
    private DcMotor xrailLift;

    private Servo clawLeft;
    private Servo clawRight;
    private Servo JewelArm;

    ModernRoboticsI2cGyro MRGyro;

    ModernRoboticsI2cColorSensor MRcolor;
    ColorSensor REVcolor;

    public int currentDegrees;

    long start = System.currentTimeMillis();

    public void drive(boolean forward, int distanceEncodeVal)
    {
        distanceEncodeVal = -(int)Math.round((distanceEncodeVal/(4*Math.PI))*1120);

        motorFrontLeft.setPower(0.3);
        motorFrontRight.setPower(0.3);
        motorBackLeft.setPower(0.3);
        motorBackRight.setPower(0.3);


        if(forward)
        {
            motorFrontLeft.setTargetPosition(-distanceEncodeVal);
            motorFrontRight.setTargetPosition(distanceEncodeVal);
            motorBackLeft.setTargetPosition(-distanceEncodeVal);
            motorBackRight.setTargetPosition(distanceEncodeVal);
        }
        else
        {
            motorFrontLeft.setTargetPosition(distanceEncodeVal);
            motorFrontRight.setTargetPosition(-distanceEncodeVal);
            motorBackLeft.setTargetPosition(distanceEncodeVal);
            motorBackRight.setTargetPosition(-distanceEncodeVal);
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
        Thread.sleep(5000);

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
        motorFrontLeft.setPower(0.2);
        motorBackLeft.setPower(0.2);
        motorFrontRight.setPower(0.2);
        motorBackRight.setPower(0.2);

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

////////////////START////////////////
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.update();
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        pinion1 = hardwareMap.dcMotor.get("pinion1");
        pinion2 = hardwareMap.dcMotor.get("pinion2");
        winch = hardwareMap.dcMotor.get("winch");
        xrailLift = hardwareMap.dcMotor.get("xrailLift");
////////////////SERVOS/////////////////
        clawLeft = hardwareMap.servo.get("clawLeft");
        clawRight = hardwareMap.servo.get("clawRight");
        JewelArm = hardwareMap.servo.get("JewelArm");

////////////////SENSORS////////////////
        MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        MRcolor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "MRcolor");

        //REVcolor = hardwareMap.colorSensor.get("REVcolor");

        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        pinion2.setDirection(DcMotorSimple.Direction.REVERSE);
        pinion1.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Zero power set
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinion1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pinion2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        xrailLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        //clawClose();

        waitForStart();
////////////////AUTONOMOUS////////////////
        clawClose();

        try{
            Thread.sleep(2000);
        }catch(Exception e){}

        blockRack(2);

        JewelArm.setPosition(0.75);
        try{
            Thread.sleep(2000);
        }catch(Exception e){}
        JewelArm.setPosition(0.86);

        try{
            Thread.sleep(2000);
        }catch(Exception e){}
        if(MRcolor.red() > 2)
        {
            telemetry.addData("I SAW:", "RED BALL");
            telemetry.update();
            turn(-10);

            JewelArm.setPosition(0.2);

            turn(0);
            telemetry.addData("done with jewel", "?>?>?>?>?>");
            telemetry.update();

            try{
                Thread.sleep(1000);
            }catch(Exception e){}

            JewelArm.setPosition(0.2);
            drive(false, 38);
        }
        else
        {
            drive(true, 5);
            try{
                Thread.sleep(1000);
            }catch(Exception e){}

            JewelArm.setPosition(0.2);

            drive(false, 33);
        }


        try{
            Thread.sleep(1000);
        }catch(Exception e){}

        blockRack(0);

        try{
            Thread.sleep(1000);
        }catch(Exception e){}

        clawOpen();

        while(opModeIsActive())
        {
            telemetry.addData("motorBackLeft", motorBackLeft.getCurrentPosition());
            telemetry.addData("motorBackRight", motorBackRight.getCurrentPosition());
            telemetry.addData("motorFrontLeft", motorFrontLeft.getCurrentPosition());
            telemetry.addData("motorFrontRight", motorFrontRight.getCurrentPosition());

            telemetry.addData("Integrated z:", MRGyro.getIntegratedZValue());

            telemetry.update();
        }
    }
}