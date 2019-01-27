package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
/**
 * Created by Grace on 1/27/2018.
 */


@Autonomous (name = "Autonomous using Main", group = "Linear Opmode_Auto")
@Disabled

public class AutonomousMain extends LinearOpMode
{

    Main mainClass = new Main();

    @Override
    public void runOpMode() throws InterruptedException
    {
        mainClass.initializeRobot();

        waitForStart();

        mainClass.drive(true, 10);

        mainClass.turn(90);

        mainClass.clawOpen();

        telemetry.addData("New autonomous", "complete");
        telemetry.update();
    }
}
