/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "TensorFlow Crater", group = "Linear OpMode")

public class TensorFlowOnlyGold extends LinearOpMode
{

    int positionFromLeft;

    int i;

    private String Date;
    private ElapsedTime runtime = new ElapsedTime();

    public int currentDegrees;

    long start = System.currentTimeMillis();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AbcVPzn/////AAABmfxPk2CjYUCDk/jUfLfUqQcCh1/8JJZxiUGFfp6nahadDcwD3ygm7soBeBWLlge6tRjJLkaZ5aY7WEcaiFx82AeJ9UxxXs2m3osSYO4PI+nn8HshdKAxR/1l0wuz0zBk+tH2p+m2EQ/jkxPQ2RA0v8j6c9AFQbhSLHQz5uTkK3pfL/A2tOEKWEz/0V1GspPfOvWj2UfaWP3FbvWrUBLy+9lofP05UHQFEc0e8lhgsBY64z9BjgbGQmoNbsGF3I3UB8xXJkcsJq8AX6pxwGKISy/UqPMZpC8f8IorgqkrocG9fZ+51kSdWm0ryB7frNsVbOvFp2eKaHIHs22JpBhK7j2dk9oTREgXYZVweQGtumxn";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    OpMode opmode;
    @Override
    public void runOpMode()
    {
        Hardware h = new Hardware();

        try {
            h.init(hardwareMap, telemetry);

        }catch(Exception e)
        {
            telemetry.addData("Something failed to initialize", ":");
            e.printStackTrace();
        }


        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        //Calibrate gyro

        h.MRGyro.calibrate();
        while(h.MRGyro.isCalibrating())
        {
            telemetry.update();
            telemetry.addData("Gyro:", "calibrating");
        }
        telemetry.addData("Calibration", "complete");
        telemetry.addData("Initialization ", "complete");
        telemetry.addData("Heading: ", h.MRGyro.getIntegratedZValue());
        telemetry.update();

        waitForStart();

        //START

        h.motorLift.setTargetPosition(13000);
        h.motorLift.setPower(1);

        h.motorArm.setTargetPosition(2000);
        h.motorArm.setPower(1);

        try{
            Thread.sleep(5000);
        }catch(Exception e){}

        h.strafe(true,10,0.4);


        h.motorLift.setTargetPosition(0);
        try{
            Thread.sleep(500);
        }catch(Exception e){}

        h.strafe(false,10, 0.4);

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive())
            {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;


                            for (Recognition recognition : updatedRecognitions) {

                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    telemetry.addData("GoldMineralX", recognition.getLeft());
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();

                                }
                            }


                            if (goldMineralX < 200) {
                                telemetry.addData("Gold:", "Left");
                                telemetry.update();

                                positionFromLeft = 1;
                            } else if (goldMineralX < 400 && goldMineralX > 200) {
                                telemetry.addData("Gold:", "Middle");
                                telemetry.update();

                                positionFromLeft = 2;
                            } else {
                                telemetry.addData("Gold:", "Right");
                                telemetry.update();

                                positionFromLeft = 3;
                            }
                        }

                }
                i++;
                idle();

                if(i > 100000){
                    break;
                    //End while loop
                }
            }
        }

        telemetry.addData("Sampling Position:", positionFromLeft);
        telemetry.update();
        
        if (tfod != null) {
            tfod.shutdown();
        }

        h.motorArm.setTargetPosition(0);

        h.drive(true, 14, 0.5);
        
        if(positionFromLeft == 1)
        {
            h.turn(40, 0.5, 0.2);
            h.drive(true, 24, 0.5);

            h.drive(false, 24, 0.5);
        }

        if(positionFromLeft == 2)
        {
            h.drive(true,20,0.5);
            h.drive(false, 20, 0.5);
        }

        if(positionFromLeft == 3)
        {
            h.turn(-40, 0.5, 0.2);
            h.drive(true, 24, 0.5);

            h.drive(false, 24, 0.5);

        }
        //END OF SAMPLING


        //Drive to wall
        h.turn(90, 0.5, 0.2);
        h.drive(true, 42, 0.8);
        
        //drive to depot
        h.turn(135, 0.5, 0.2);
        h.drive(true, 40, 1);

        try{
            Thread.sleep(1000);
        }catch(Exception e){}

        //drop off marker
        h.bucketFlapServo.setPosition(0.2);
        h.drive(true, 2, 1);

        //drive to crater
        try{
            Thread.sleep(2000);
        }catch(Exception e){}

        h.drive(false, 70, 1);




        while (opModeIsActive())
        {
            //Should keep auto running
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
