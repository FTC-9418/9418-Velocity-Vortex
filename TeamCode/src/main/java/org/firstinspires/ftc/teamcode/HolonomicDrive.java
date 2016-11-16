/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;


import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



/**
 * This file provides basic Telop driving for a Holonomic drivetrain robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common hardware class to define the devices on the robot.
 * All device access is managed through the Hardware class.
 *
 * This particular OpMode executes a holonomic drive Teleop for the robot
 * It raises and lowers a arm using the Gampad Y and A buttons respectively.
 * It also uses a servo connected to a rack and pinion system to push buttons on the beacon
 *
 */

@TeleOp(name="Holonomic Drive", group="Drive")
public class HolonomicDrive extends OpMode{

    /* Declare OpMode members. */
    Hardware robot = new Hardware(); // Use the class created to define the robot's hardware

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        //telemetry.addData("Test", "Foo Bar Fizz Buzz Xyzzy");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drive();
        winch();
        push();
        color();
    }

    public void drive(){
        double x; // Right/ Left Movement
        double y; // Forward/ Backward movement
        double z; // Used for turning
        x = -gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        z = -gamepad1.right_stick_x;

        if(y >= 0.5) {
            robot.fr.setPower(0.5);
            robot.fl.setPower(0.5);
            robot.br.setPower(0.5);
            robot.bl.setPower(0.5);
        } else if(y <= -0.5) {
            robot.fr.setPower(-0.5);
            robot.fl.setPower(-0.5);
            robot.br.setPower(-0.5);
            robot.bl.setPower(-0.5);
        } else if(x >= 0.5) {
            robot.fr.setPower(0.5);
            robot.fl.setPower(-0.5);
            robot.br.setPower(-0.5);
            robot.bl.setPower(0.5);
        } else if(x <= -0.5) {
            robot.fr.setPower(-0.5);
            robot.fl.setPower(0.5);
            robot.br.setPower(0.5);
            robot.bl.setPower(-0.5);
        } else if(z >= 0.5) {
            robot.fr.setPower(0.5);
            robot.fl.setPower(-0.5);
            robot.br.setPower(0.5);
            robot.bl.setPower(-0.5);
        } else if(z <= -0.5) {
            robot.fr.setPower(-0.5);
            robot.fl.setPower(0.5);
            robot.br.setPower(-0.5);
            robot.bl.setPower(  0.5);
        } else {
            robot.fr.setPower(0);
            robot.fl.setPower(0);
            robot.br.setPower(0);
            robot.bl.setPower(0);
        }
    }

    public void winch() {
        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad1.y) {
            robot.wl.setPower(1);
            robot.wr.setPower(1);
        } else if (gamepad1.a) {
            robot.wl.setPower(-1);
            robot.wr.setPower(-1);
        } else {
            robot.wl.setPower(0);
            robot.wr.setPower(0);
        }
    }

    public void push() {
        if(gamepad1.left_bumper) {
            robot.push.setPosition(1);
        } else if(gamepad1.right_bumper) {
            robot.push.setPosition(-1);
        } else {
            robot.push.setPosition(0.5);
        }
    }

    public void color() {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        // send the info back to driver station using telemetry function.

        //telemetry.addData("LED", bLedOn ? "On" : "Off");
        Color.RGBToHSV(robot.beacon.red() * 8, robot.beacon.green() * 8, robot.beacon.blue() * 8, hsvValues);

        telemetry.addData("Clear", robot.beacon.alpha());
        telemetry.addData("Red  ", robot.beacon.red());
        telemetry.addData("Green", robot.beacon.green());
        telemetry.addData("Blue ", robot.beacon.blue());
        telemetry.addData("Hue", hsvValues[0]);
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
