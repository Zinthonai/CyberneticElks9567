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

/**
 * Created by Grace on 1/27/2018.
 */

@TeleOp (name = "TeleOp Main", group = "Linear OpMode")
@Disabled
public class TeleOpMain extends LinearOpMode
{
    Main mainClass = new Main();

    @Override
    public void runOpMode() throws InterruptedException
    {
        mainClass.initializeRobot();

        waitForStart();

        mainClass.leftPower = Math.pow(gamepad1.left_stick_y, 3/2);
        mainClass.rightPower = Math.pow(gamepad1.left_stick_y, 3/2);
        mainClass.rotation = Math.pow(-gamepad1.right_stick_x, 3/2);

        mainClass.motorBackRight.setPower(Range.clip(mainClass.rightPower, -1,1) - mainClass.rotation);
        mainClass.motorFrontRight.setPower(Range.clip(mainClass.rightPower,-1,1)- mainClass.rotation);
        mainClass.motorFrontLeft.setPower(Range.clip(mainClass.leftPower,-1, 1)+ mainClass.rotation);
        mainClass.motorBackLeft.setPower(Range.clip(mainClass.leftPower, -1, 1) + mainClass.rotation);

    }
}
