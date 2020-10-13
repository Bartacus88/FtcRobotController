/* Copyright (c) 2017 FIRST. All rights reserved.
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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Sensor: MR Color Bart", group = "Sensor")
//@Disabled
public class BartColorSensor extends LinearOpMode {

  ColorSensor colorSensor;    // Hardware Device Object
  private static long prev_sensor_millis= 0;
  private static long prev_actuator_millis= 0;
  private static long prev_telemetry_millis= 0;
  private static long SENSOR_PERIOD = 25; // milliseconds
  private static long ACTUATOR_PERIOD = 25;
  private static long TELEMETRY_PERIOD = 250;

  private static boolean white_detected = false;

  private boolean colorWhiteDetected(int red_color, int green_color, int blue_color)
  {
    if (red_color >= 900 && green_color >= 900 && blue_color >= 900)
    {
      return (true);
    }
    else
    {
      return (false);
    }
  }

  @Override
  public void runOpMode() {
    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    // get a reference to our ColorSensor object.
    colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

    // Set the LED on
    colorSensor.enableLed(true);

    // wait for the start button to be pressed.
    waitForStart();

    // while the op mode is active, loop and read the RGB data.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive())
    {
      long millis = System.currentTimeMillis();

      if (millis - prev_sensor_millis >= SENSOR_PERIOD)
      {
        prev_sensor_millis = millis;
        white_detected = colorWhiteDetected(colorSensor.red(), colorSensor.green(), colorSensor.blue());
      }
      if (millis - prev_telemetry_millis >= TELEMETRY_PERIOD)
        {
          prev_telemetry_millis = millis;
          // send the info back to driver station using telemetry function.
          //telemetry.addData("LED", bLedOn ? "On" : "Off");
          //telemetry.addData("Clear", colorSensor.alpha());
          telemetry.addData("Red  ", colorSensor.red());
          telemetry.addData("Green", colorSensor.green());
          telemetry.addData("Blue ", colorSensor.blue());
          //telemetry.addData("Hue", hsvValues[0]);
          //telemetry.addData("Millis", millis);
          telemetry.addData("colorWhite ", white_detected);
        }

      // change the background color to match the color detected by the RGB sensor.
      // pass a reference to the hue, saturation, and value array as an argument
      // to the HSVToColor method.
      relativeLayout.post(new Runnable() {
        public void run() {
          //relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
        }
      });

      telemetry.update();
    }

    // Set the panel back to the default color
    relativeLayout.post(new Runnable() {
      public void run() {
        relativeLayout.setBackgroundColor(Color.WHITE);
      }
    });
  }
}
