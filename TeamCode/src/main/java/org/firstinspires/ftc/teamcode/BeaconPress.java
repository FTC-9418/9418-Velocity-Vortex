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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
  private int beacon_threshold = 3;
  private float floor_threshold = 0.3f;
  private int driveTime = 2500;
  private int searchDirection = Hardware.Direction_Left;
  private int hitBallDirection = Hardware.Direction_ReverseRight;
  private int rotateDirection = Hardware.Direction_RotateRight;
  private int unRotateDirection = Hardware.Direction_RotateLeft;
  private int rotateTime = 800;

  public BeaconPress(boolean lookForRed) {
      this.lookForRed = lookForRed;
    if (lookForRed){
      this.searchDirection = Hardware.Direction_Right;
      this.hitBallDirection = Hardware.Direction_ReverseLeft;
      this.driveTime = 2800;
      this.rotateTime = 400;
    }
  }

  @Override
  public void runOpMode() {

    Hardware robot = new Hardware();
    robot.init(hardwareMap);

    // wait for the start button to be pressed.
    telemetry.addData("Mode ", "waiting...");
    telemetry.update();


    waitForStart();

    telemetry.update();
    robot.drive(Hardware.Direction_Forward | searchDirection, 0.5);
    sleep(driveTime);
    findWall(robot);

    robot.stop();

    // loop and read the RGB data.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    int presses = 0;
    while (opModeIsActive() && presses < 2)  {



      telemetry.addData("Mode ", "Search...");
      telemetry.addData("Red ", robot.beacon.red());
      telemetry.addData("Blu ", robot.beacon.blue());
      telemetry.update();

      if (robot.isFloorLineDetected(floor_threshold)){
        telemetry.addData("Light: ", robot.floor.getLightDetected());
        pressBeacon(robot);
        presses++;
        if (presses < 2) {
          robot.drive(Hardware.Direction_Reverse, 0.3);
          sleep(200);
          robot.drive(searchDirection, 0.5);
          sleep(500);
        } else if (presses == 2) {
          postBeacon(robot);
          break;
        }
      }

      boolean foundLine = false;


      if (!robot.touch.isPressed()){
        foundLine = findWall(robot);
      }

      if(!foundLine){
        robot.drive(searchDirection, 0.3);
      }
    }
  }

  private void postBeacon(Hardware robot) {

    robot.drive(hitBallDirection, .7);
    sleep(1700);
    robot.drive(rotateDirection, .7);
    sleep(rotateTime);
    robot.stop();
    robot.firetwice();
    sleep(500);
    robot.drive(unRotateDirection, .7);
    sleep(rotateTime);
    robot.drive(hitBallDirection, .7);
    sleep(1200);
    robot.stop();
  }

  private boolean findWall(Hardware robot) {
    robot.drive(Hardware.Direction_Forward, 0.2);
    while(!robot.touch.isPressed() && opModeIsActive()) {
      telemetry.addData("Mode ", "Find Wall...");
      telemetry.addData("Touch ", robot.touch.isPressed());
      telemetry.update();
      sleep(10);
    }
    return robot.isFloorLineDetected(floor_threshold);
  }

  private void pressBeacon(Hardware robot) {
    findWall(robot);
    robot.stop();
    if (lookForRed) {
      if (robot.beacon.blue() > beacon_threshold) {
        robot.push.setPosition(1);
      } else {
        robot.push.setPosition(0);
      }
    } else {
      if (robot.beacon.blue() > beacon_threshold) {
        robot.push.setPosition(0);
      } else {
        robot.push.setPosition(1);
      }
    }
    findWall(robot);
    robot.stop();

    sleep(1000);
    robot.push.setPosition(0.5);
  }
}



