package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "CardBot3000", group = "TeleOp")
public class CardBot3000 extends LinearOpMode
{
    OpMode opmode;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor armMotor;

    Servo wrist;
    Servo finger1;


    @Override
    public void runOpMode()
    {

        try {
            motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
            motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
            armMotor = hardwareMap.dcMotor.get("armMotor");
            wrist = hardwareMap.servo.get("wrist");
            finger1 = hardwareMap.servo.get("finger1");

        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            e.printStackTrace();
        }


        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Initialization ", "complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            motorBackLeft.setPower(gamepad1.left_stick_y);
            motorBackRight.setPower(gamepad1.right_stick_y);

            if (gamepad1.dpad_up)
            {
                armMotor.setPower(.5);
            }

            if (gamepad1.dpad_down)
            {
                armMotor.setPower(-.5);
            }
            if (!gamepad1.dpad_up && !gamepad1.dpad_down)
            {
                armMotor.setPower(0);
            }

            if (gamepad1.dpad_left)
            {
                while (gamepad1.dpad_left)
                {
                    wrist.setPosition(0);
                }
                finger1.setPosition(finger1.getPosition());

            }
            if (gamepad1.dpad_right)
            {
                while(gamepad1.dpad_right)
                {
                    wrist.setPosition(1);
                }
                finger1.setPosition(finger1.getPosition());
            }
            if (gamepad1.left_bumper)
            {
                while (gamepad1.left_bumper)
                {
                    finger1.setPosition(0);
                }
                finger1.setPosition(finger1.getPosition());
            }
            if (gamepad1.right_bumper)
            {
                while (gamepad1.right_bumper)
                {
                    finger1.setPosition(.25);
                }
                finger1.setPosition(finger1.getPosition());

            }

            //finger1 is 0 to 0.25
            //wrist is 0 to 1
            /*
            wrist.setPosition(gamepad2.left_stick_y);
            finger1.setPosition(gamepad2.right_stick_y);
            */



            telemetry.addData("wrist position:", wrist.getPosition());
            telemetry.addData("finger1 position:", finger1.getPosition());
            telemetry.update();
        }

    }
}
