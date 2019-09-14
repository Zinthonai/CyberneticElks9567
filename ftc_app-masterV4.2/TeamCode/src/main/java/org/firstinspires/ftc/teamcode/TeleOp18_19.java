package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "TeleOp Rover Ruckus", group = "TeleOp")
public class TeleOp18_19 extends LinearOpMode
{
    OpMode opmode;

    private int currentChainPos;

    private boolean liftToTop = false;
    private boolean isRaisingArmUp = false;
    private boolean isRaisingArmDown = false;

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
            if ((!gamepad1.dpad_up && !gamepad1.dpad_down) && !isRaisingArmDown && !isRaisingArmUp)
            {
                h.motorWinch.setTargetPosition(h.motorWinch.getCurrentPosition());
            }

////////SPINNER
            if (gamepad1.a) {
                //Pull in minerals
                h.motorSpinner.setPower(-1);
            }
            if (gamepad1.b) {
                //Spit out extra minerals
                h.motorSpinner.setPower(0.6);
            }
            if (gamepad1.x) {
                h.motorSpinner.setPower(0);
            }

////////MANUAL ARM
            if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0) {

                //LIFT ARM

                isRaisingArmUp = false; //Stop automatic raising if manual is tried
                isRaisingArmDown = false;

                h.motorArm.setPower(1);
                h.motorArm.setTargetPosition(11000);

                while (gamepad1.right_trigger > 0 && opModeIsActive()) {
                }
                currentChainPos = h.motorArm.getCurrentPosition();
            }
            if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0) {

                //DROP ARM

                isRaisingArmUp = false; //Stop automatic raising if manual is tried
                isRaisingArmDown = false;

                h.motorArm.setPower(1);
                h.motorArm.setTargetPosition(0);

                while (gamepad1.left_trigger > 0 && opModeIsActive()) {
                }
                currentChainPos = h.motorArm.getCurrentPosition();
            }
            if ((gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) && !isRaisingArmDown && !isRaisingArmUp)
            {
                h.motorArm.setTargetPosition(currentChainPos);
            }

////////MANUAL ARM END

////////AUTO ARM
            if (gamepad1.left_bumper)
            {
                isRaisingArmUp = false;
                isRaisingArmDown = true;
            }
            if (gamepad1.right_bumper)
            {
                isRaisingArmUp = true;
                isRaisingArmDown = false;
            }
            if(isRaisingArmUp)
            {
                h.motorArm.setPower(1);
                h.motorWinch.setPower(1);
                if(h.motorArm.getCurrentPosition() < 2000)
                {


                    //h.motorWinch.setTargetPosition(0);
                    h.motorArm.setTargetPosition(4500);
                }
                /*if(h.motorArm.getCurrentPosition() > 2000 && h.motorArm.getCurrentPosition() < 4000)
                {

                    h.motorWinch.setTargetPosition(1500);
                    h.motorArm.setTargetPosition(4500);
                }*/
                if(h.motorArm.getCurrentPosition() > 4000 && h.motorArm.getCurrentPosition() < 11000)
                {

                    h.motorWinch.setTargetPosition(3600);
                    h.motorArm.setTargetPosition(10500);
                }
            }
            if(isRaisingArmDown)
            {
                h.motorArm.setPower(1);
                h.motorWinch.setPower(1);

                if(h.motorArm.getCurrentPosition() > 3000)
                {
                    h.motorWinch.setTargetPosition(2500);
                    h.motorArm.setTargetPosition(2500);
                }


                /*if(h.motorArm.getCurrentPosition() > 2000 && h.motorArm.getCurrentPosition() < 10000)
                {
                    h.motorWinch.setTargetPosition(1500);
                    h.motorArm.setTargetPosition(1500);
                }*/
                if(h.motorArm.getCurrentPosition() < 3000)
                {
                    h.motorWinch.setTargetPosition(0);
                    h.motorArm.setTargetPosition(0);
                }


            }
////////AUTO ARM END

////////ACTUATOR
            if (gamepad1.dpad_left && h.motorLift.getCurrentPosition() < 13000)
            {
                h.motorLift.setPower(1);
                h.motorLift.setTargetPosition(13000);
            }
            if(gamepad1.right_stick_button)
            {
                liftToTop = true;
            }
            if(liftToTop)
            {
                h.motorLift.setPower(1);
                h.motorLift.setTargetPosition(13000);
                if(h.motorLift.getCurrentPosition() >= 13000)
                {
                    liftToTop = false;
                }
            }
            if (gamepad1.dpad_right && h.motorLift.getCurrentPosition() > 0)
            {
                h.motorLift.setPower(1);
                h.motorLift.setTargetPosition(0);
            }
            if (!gamepad1.dpad_left && !gamepad1.dpad_right && !liftToTop)
            {
                h.motorLift.setPower(0);
            }

///////////FLAP SERVO

            if(gamepad1.y)
            {
                h.bucketFlapServo.setPosition(0.5);
            }
            else
            {
                h.bucketFlapServo.setPosition(0.8);
            }


            //For testing
            //h.bucketFlapServo.setPosition(gamepad2.right_stick_y);

        }
    }
}

