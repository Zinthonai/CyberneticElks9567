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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

//import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "TeleOp Staggered Block Arm--THIS ONE", group = "Linear Opmode")
@Disabled
public class MyTeleOpStaggeredBlockArm extends LinearOpMode
{
////////////////////////////////////////////////////////////////////////////////////////////////////
    //Motors, servos, sensors, and variable are set up here
////////////////////////////////////////////////////////////////////////////////////////////////////

    //This allows us to see how long our program has bee running
    private String Date;
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor motorBackLeft;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorFrontRight;

    DcMotor pinion1;
    DcMotor pinion2;

    DcMotor winch;
    DcMotor xrailLift;

    Servo clawLeft;
    Servo clawRight;
    Servo JewelArm;
    Servo xClaw;

    ModernRoboticsI2cColorSensor MRcolor;
    ModernRoboticsI2cGyro MRGyro;
    ModernRoboticsI2cRangeSensor MRRange;

    double leftPower;
    double rightPower;

    double rightRotation;
    double rotation;

    int blockArmPos;

    boolean MRUltraDisabled;

    @Override
    public void runOpMode()
    {
////////////////////////////////////////////////////////////////////////////////////////////////////
        //Motors/servos/sensors initialized
////////////////////////////////////////////////////////////////////////////////////////////////////
        telemetry.update();

////////////////////////////////////////////////////////////////////////////////////////////////////
        //This block here makes sure our program doesn't crash on startup if something
        //isn't plugged in.
////////////////////////////////////////////////////////////////////////////////////////////////////
        try
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
            xClaw = hardwareMap.servo.get("xClaw");

            MRcolor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "MRcolor");
            MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
            MRRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "MRRange");

        } catch (Exception e) {
            //telemetry.addData("Init Error:", "Something failed to initialize");
        }

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////Reverse the backwards facing motors so that motors all act the same
////////////////////////////////////////////////////////////////////////////////////////////////////
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        pinion2.setDirection(DcMotorSimple.Direction.REVERSE);



////////////////////////////////////////////////////////////////////////////////////////////////////
        //Tell motors to use encoders so we can control exactly what they do
////////////////////////////////////////////////////////////////////////////////////////////////////
        pinion1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pinion2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xrailLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pinion1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pinion2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pinion1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pinion2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        xrailLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //telemetry.addData("SETUP STATUS:", "COMPLETE");
        //telemetry.update();

////////////////////////////////////////////////////////////////////////////////////////////////////
        //Wait for the play button to be pressed
////////////////////////////////////////////////////////////////////////////////////////////////////
        waitForStart();

        while (opModeIsActive())
        {
////////////////////////////////////////////////////////////////////////////////////////////////////
            //Update the data being sent to the phone for our use.
            //We use this to troubleshoot sensors or motors etc...
////////////////////////////////////////////////////////////////////////////////////////////////////
          //telemetry.update();
          ////telemetry.addData("Status", "Run Time: " + runtime.toString());
          ////telemetry.addData("clawRight Position:", clawRightPos);
          //telemetry.addData("pinion1", pinion1.getCurrentPosition());
          //telemetry.addData("pinion2", pinion2.getCurrentPosition());
          //telemetry.addData("winch", winch.getCurrentPosition());
          //telemetry.addData("xrailLift", xrailLift.getCurrentPosition());
          //telemetry.addData("Range:", MRRange.cmUltrasonic());
          //telemetry.addData("Is the ultrasonic sensor disabled", MRUltraDisabled);
          
          ////telemetry.addData("gyro:", MRGyro.getHeading());

            //telemetry.addData("block Position:", blockArmPos);

////////COLOR SENSOR////////
////////////////////////////////////////////////////////////////////////////////////////////////////
          //Color sensor data for the various colors
////////////////////////////////////////////////////////////////////////////////////////////////////
          //telemetry.addData("RED:", MRcolor.red());
          //telemetry.addData("BLUE:", MRcolor.blue());

////////THROTTLE DRIVE////////

////////////////////////////////////////////////////////////////////////////////////////////////////
            //Only sets the joystick to the motor power if we are not turning.
            //This stops the motors from receiving 2 instructions at the same time, which makes them jitter.
////////////////////////////////////////////////////////////////////////////////////////////////////
            leftPower = Math.pow(gamepad1.left_stick_y, 3/2);
            rightPower = Math.pow(gamepad1.left_stick_y, 3/2);

            rotation = Math.pow(-gamepad1.right_stick_x, 3/2)/1.5;

            //telemetry.addData("leftPower", leftPower);
            //telemetry.addData("rightPower", rightPower);

            telemetry.update();
            telemetry.addData("motorbackleft", motorBackLeft.getPower());
            telemetry.addData("motorbackright", motorBackRight.getPower());
            telemetry.addData("motorfrontleft", motorFrontLeft.getPower());
            telemetry.addData("motorfrontright", motorFrontRight.getPower());
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            telemetry.update();


/*
            if(Math.abs(rotation) > 0)
            {
                motorBackRight.setPower(Range.clip(rightPower, -0.7, 0.7) - rotation);
                motorFrontRight.setPower(Range.clip(rightPower, -0.7, 0.7) - rotation);
                motorFrontLeft.setPower(Range.clip(leftPower, -0.7, 0.7) + rotation);
                motorBackLeft.setPower(Range.clip(leftPower, -0.7, 0.7) + rotation);
            }
            else
            {
                motorBackRight.setPower(Range.clip(rightPower, -1, 1) - rotation);
                motorFrontRight.setPower(Range.clip(rightPower, -1, 1) - rotation);
                motorFrontLeft.setPower(Range.clip(leftPower, -1, 1) + rotation);
                motorBackLeft.setPower(Range.clip(leftPower, -1, 1) + rotation);
            }
*/
////////////////////////////////////////////////////////////////////////////////////////////////////
            /*
            if(gamepad2.right_trigger > 0.01f)
            {
                motorBackRight.setPower(gamepad2.right_trigger/ 4);
                motorBackLeft.setPower(-gamepad2.right_trigger/ 4);
                motorFrontLeft.setPower(-gamepad2.right_trigger/ 4);
                motorFrontRight.setPower(gamepad2.right_trigger/ 4);
            }

            if(gamepad2.left_trigger > 0.01f)
            {
                motorBackRight.setPower(-gamepad2.left_trigger / 4);
                motorBackLeft.setPower(gamepad2.left_trigger / 4);
                motorFrontLeft.setPower(gamepad2.left_trigger / 4);
                motorFrontRight.setPower(-gamepad2.left_trigger / 4);
            }
*/
////////X-RAIL////////

////////////////////////////////////////////////////////////////////////////////////////////////////
            //Winds up the winch to extend the X-rail slide when the d-pad up and down are pressed
////////////////////////////////////////////////////////////////////////////////////////////////////

            if(gamepad2.start)
            {
                if(MRUltraDisabled)
                {
                    MRUltraDisabled= false;
                }
                else
                {
                    MRUltraDisabled = true;
                }

            }

            if(MRUltraDisabled)
            {
                if (gamepad2.dpad_up && winch.getCurrentPosition() < 8200) {
                    winch.setTargetPosition(8200);
                    winch.setPower(1);
                    winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (gamepad2.dpad_up && winch.getCurrentPosition() > 0) {
                    }
                    winch.setPower(0);
                }
            }
            else
            {
                if (gamepad2.dpad_up/* && winch.getCurrentPosition() < 8200 && MRRange.cmUltrasonic() > 6*/)
                {
                    //winch.setTargetPosition(8200);
                    winch.setPower(1);
                    //winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (gamepad2.dpad_up/* && winch.getCurrentPosition() > 0 && MRRange.cmUltrasonic() > 6*/)
                    {
                    }
                    winch.setPower(0);
                }
            }

            if (gamepad2.dpad_down)
            {
                //winch.setTargetPosition(0);
                winch.setPower(-1);
                //winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (gamepad2.dpad_down)
                {
                }
                winch.setPower(0);
            }
////////////////////////////////////////////////////////////////////////////////////////////////////
            //Raises and lowers the X-rail slide when the d-pad left and right are pressed
////////////////////////////////////////////////////////////////////////////////////////////////////
            if (gamepad2.right_bumper)
            {
                xrailLift.setTargetPosition(2200);
                xrailLift.setPower(1);
                xrailLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (gamepad2.right_bumper && xrailLift.getCurrentPosition() < 2200 && xrailLift.getCurrentPosition() > 0)
                {
                }
                xrailLift.setPower(0);
            }
            if (gamepad2.left_bumper)
            {
                xrailLift.setTargetPosition(0);
                xrailLift.setPower(1);
                xrailLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (gamepad2.left_bumper && xrailLift.getCurrentPosition() > 0)
                {
                }
                xrailLift.setPower(0);
            }

////////////////////////////////////////////////////////////////////////////////////////////////////
            //Opens and closes the relic grabber servo with the a and b buttons on controller 2
////////////////////////////////////////////////////////////////////////////////////////////////////
            if(gamepad2.a)
            {
                //Open
                xClaw.setPosition(1);
            }
            if(gamepad2.b)
            {
                //Close
                xClaw.setPosition(0.4);
            }
////////RACK&PINION////////

////////////////////////////////////////////////////////////////////////////////////////////////////
            //Raises the block claw up and down the column at the touch of the right and left bumper
////////////////////////////////////////////////////////////////////////////////////////////////////

            //UP
            pinion1.setPower(1);
            pinion2.setPower(0.7);

            switch (blockArmPos)
            {
                case 0:
                    pinion1.setTargetPosition(0);
                    pinion2.setTargetPosition(0);
                    break;

                case 1:
                    pinion1.setTargetPosition(1100);
                    pinion2.setTargetPosition(0);
                    break;

                case 2:
                    pinion1.setTargetPosition(1100);
                    pinion2.setTargetPosition(1250);
                    break;

                case 3:
                    pinion1.setTargetPosition(1100);
                    pinion2.setTargetPosition(2800);
                    break;
            }

            pinion1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pinion2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (gamepad1.right_bumper && blockArmPos < 3)
            {
                blockArmPos++;
            }
            //DOWN
            if (gamepad1.left_bumper && blockArmPos > 0)
            {
                blockArmPos--;
            }

////////CLAW////////
////////////////////////////////////////////////////////////////////////////////////////////////////
            //Opens and closes the block grabber servos with controller 1 a and b buttons
            //A is open. B is closed.
////////////////////////////////////////////////////////////////////////////////////////////////////
            if (gamepad1.a)
            {
                clawLeft.setPosition(0);
            }
            if (gamepad1.a)
            {
                clawRight.setPosition(0.25);
            }
            if (gamepad1.b)
            {
                clawLeft.setPosition(0.25);
            }
            if (gamepad1.b)
            {
                clawRight.setPosition(0);
            }
////////JewelArm////////
            if(gamepad2.x)
            {
                JewelArm.setPosition(0);
            }
            if(gamepad2.y)
            {
                JewelArm.setPosition(1 );
            }

            //telemetry.update();
        }
        ////////STOP////////
////////////////////////////////////////////////////////////////////////////////////////////////////
        //This is where the program goes when the stop button is pressed
////////////////////////////////////////////////////////////////////////////////////////////////////

    }
}
