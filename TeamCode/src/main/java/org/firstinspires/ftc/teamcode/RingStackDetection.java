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
 *
 *
 *
 * UPDATE:  Modified from sample file for Kernel Panic and Juden Ki
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the skystones.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
//@Disabled
public class RingStackDetection {
    private LinearOpMode myOpMode = null;
    private boolean timeLeft = false;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    public enum Rings
    {
        NONE, //0
        ONE,     //1
        FOUR,    //2
    }

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
    public static final String VUFORIA_KEY = "Afsw2oP/////AAABmfzepGuVnkr3uHCSQlCl89hgf8A+n9yEsMhH8pfA7Ttz/JfeOCGHzGmHjZMt0IHzaR5tUbVK4L59qd0RJsjAfrmrCsu/CDMm90dy3T9+eRo+zlw6aGRHD0j3EhngUWY0dc1PrRZpWXa+KCLOy3rtB+aWaZDBxILq6uCiqRtLGUwBTrDGQVH/fE1z32YZbDISS5F6actiiu9RhSyU8DKn1EEWeuTj0W+O7lAoDUIhJfJ0B0Iqk73Gfuv2ytbUq89obq9ZVMUqjq9rFsYiztVPOXQkWZsnv8P+eN/0XEgDUmQCU4XBABUw7bTsn9WW/xMDyqnuUMy+AbD/ag5+EPpQaAEGa9nT6n/ATK/Znu47ZlAe";
    private  VuforiaLocalizer vuforia;
    private  TFObjectDetector tfod;
    public Rings retVal = Rings.NONE;


    public Rings detectRingStack(long timelimit) {
        long quitTime;

        // Verify that configure has been called first.
        if (myOpMode == null) {
            return (retVal);
        }

        if (myOpMode.opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            // Calculate the timelimit data
            quitTime = System.currentTimeMillis() + timelimit;
            timeLeft = true;


            while (myOpMode.opModeIsActive() && timeLeft) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      myOpMode.telemetry.clear();
                      myOpMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                      if (updatedRecognitions.size() > 0) {
                        for (Recognition recognition : updatedRecognitions) {
                          myOpMode.telemetry.addData("OBJECT: ", recognition.getLabel());
                          if(recognition.getLabel() == "Quad")
                          {
                              retVal = Rings.FOUR;
                          }
                          else if (recognition.getLabel() == "Single")
                          {
                              retVal = Rings.ONE;
                          }
                          else
                          {
                              retVal = Rings.NONE;
                          }
                          myOpMode.telemetry.addData("LEFT START: ", recognition.getLeft());
                          myOpMode.telemetry.addData("RetVal: ", retVal);
                          myOpMode.telemetry.update();
                        }
                      }
                      //myOpMode.sleep(200); //Not sure that this was ever needed.
                    }
                }
                if (System.currentTimeMillis() > quitTime) {
                    timeLeft = false;
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        return (retVal);
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
        //Comment/uncomment the next two lines for an external webcam vs a phones back camera.
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }



    public void configureDetection (LinearOpMode opMode) {
        myOpMode = opMode;


        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.


        initVuforia();
        initTfod();
/* I dont understand why the below code does not compile.  But Bart's home test setup works with this.
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            myOpMode.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

 */

    }
}
