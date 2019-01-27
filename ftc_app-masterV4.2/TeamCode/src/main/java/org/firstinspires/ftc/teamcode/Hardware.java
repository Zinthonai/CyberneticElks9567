package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware
{
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorFrontLeft = null;

    public DcMotor motorArm = null;
    public DcMotor motorWinch = null;
    public DcMotor motorLift = null;
    public DcMotor motorSpinner = null;

    HardwareMap map = null;

    DcMotor.RunMode initialMode = null;

    public void init(HardwareMap aMap)
    {
        motorFrontRight = aMap.dcMotor.get("motorFrontRight");
        motorBackRight = aMap.dcMotor.get("motorBackRight");
        motorFrontLeft = aMap.dcMotor.get("motorBackLeft");
        motorFrontLeft = aMap.dcMotor.get("motorFrontLeft");

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

    public void

    public Hardware(DcMotor.RunMode enteredMode)
    {

    }

}
