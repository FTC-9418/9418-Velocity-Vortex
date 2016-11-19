/* Copyright (c) 2015 Qualcomm Technologies Inc

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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a legacy (NXT-compatible) Hitechnic Color Sensor v2.
 * It assumes that the color sensor is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
public abstract class BeaconPress extends LinearOpMode {

  private boolean lookForRed = false;
  private int threshold = 3;

  public BeaconPress(boolean lookForRed) {
      this.lookForRed = lookForRed;
  }

  @Override
  public void runOpMode() {

    Hardware robot = new Hardware();
    robot.init(hardwareMap);

    // wait for the start button to be pressed.
    telemetry.addData("Mode ", "waiting...");
    telemetry.update();
    wiggle(robot, 100);

    waitForStart();

    telemetry.addData("Mode ", "drive fwd");
    telemetry.update();
    robot.drive(Hardware.Direction_Forward, 0.1);
    sleep(250);

    telemetry.addData("Mode ", "drive diag");
    telemetry.update();
    robot.drive(Hardware.Direction_ForwardLeft, 0.05);
    sleep(1000);
    findWall(robot);

    robot.stop();
    sleep(3000);

    // loop and read the RGB data.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    int presses = 0;
    while (opModeIsActive() && presses < 2)  {

      robot.drive(Hardware.Direction_Left, 0.1);
      telemetry.addData("Mode ", "Search...");
      telemetry.addData("Red ", robot.beacon.red());
      telemetry.addData("Blu ", robot.beacon.blue());
      telemetry.update();

      if (robot.isAnyBeaconLight(threshold)){
        telemetry.addData("Mode ", "Pressing");
        pressBeacon(robot);
        presses++;
        if (presses < 2) {
          robot.drive(Hardware.Direction_Left, 0.5);
          sleep(1000);
        }
      }

    }
  }

  private void wiggle(Hardware robot, int t) {
    robot.push.setPosition(-1);
    sleep(t);
    robot.push.setPosition(1);
    sleep(t);
    robot.push.setPosition(-1);
    sleep(t);
    robot.push.setPosition(1);
    sleep(t);
    robot.push.setPosition(-1);
    sleep(t);
    robot.push.setPosition(1);
    sleep(t);
    robot.push.setPosition(0.5);
  }

  private void findWall(Hardware robot) {
    robot.drive(Hardware.Direction_Forward, 0.5);
    while(!robot.touch.isPressed() && opModeIsActive()) {
      telemetry.addData("Mode ", "Find Wall...");
      telemetry.addData("Touch ", robot.touch.isPressed());
      telemetry.update();
      sleep(10);
    }
  }

  private void pressBeacon(Hardware robot) {
    robot.stop();
    if (lookForRed) {
      if (robot.beacon.red() > threshold) {
        robot.push.setPosition(-1);
      } else {
        robot.push.setPosition(1);
      }
    } else {
      if (robot.beacon.blue() > threshold) {
        robot.push.setPosition(-1);
      } else {
        robot.push.setPosition(1);
      }
    }
    robot.drive(Hardware.Direction_Forward, 0.2);
    sleep(500);
    robot.stop();

    sleep(3000);
    robot.push.setPosition(0.5);
  }


}
