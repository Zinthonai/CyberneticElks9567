package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "CardBot3000", group = "TeleOp")
public class CardBot3000 extends LinearOpMode
{
    OpMode opmode;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;

    @Override
    public void runOpMode()
    {

        try {
            motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
            motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            e.printStackTrace();
        }

        telemetry.addData("Initialization ", "complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            motorBackLeft.setPower(gamepad1.left_stick_y);
            motorBackRight.setPower(gamepad1.right_stick_y);
        }

    }
}
