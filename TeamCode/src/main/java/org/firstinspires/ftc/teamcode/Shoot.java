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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name = "Shoot Only", group = "Auto")
public class Shoot extends LinearOpMode {

  private int initialDriveTime = 1200;
  private int finalDriveTime   = 1000;


  public Shoot() {

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
    robot.drive(Hardware.Direction_Left, 0.5);
    sleep(initialDriveTime);
    robot.drive(Hardware.Direction_Stop, 0);

    robot.initCatapult();
    primeTrigger(robot);

    robot.fireCatapult();

    robot.intake.setPower(1);
    primeTrigger(robot);

    sleep(1500);

    robot.intake.setPower(0);

    robot.fireCatapult();
    primeTrigger(robot);

    robot.drive(Hardware.Direction_Left, 0.5);
    sleep(finalDriveTime);
    robot.drive(Hardware.Direction_Stop, 0);

  }

  private void primeTrigger(Hardware robot) {
    while(!robot.primeTrigger() && !isStopRequested()) {
      sleep(10);
    }
  }

}
