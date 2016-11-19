package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 */
public class Hardware
{
    public DcMotor fl = null;
    public DcMotor bl = null;
    public DcMotor fr = null;
    public DcMotor br = null;
    public DcMotor wl = null;
    public DcMotor wr = null;
    public Servo push = null;
    public ColorSensor beacon;
    public ColorSensor floor;
    public TouchSensor touch;

    //public ModernRoboticsI2cColorSensor

    public static final double MID_SERVO =  0.5 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  =  new ElapsedTime();

    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Drivetrain Motors
        fr = hwMap.dcMotor.get("FR");
        fl = hwMap.dcMotor.get("FL");
        br = hwMap.dcMotor.get("BR");
        bl = hwMap.dcMotor.get("BL");

        // Define and Initialize Winch Motors
        wr = hwMap.dcMotor.get("WR");
        wl = hwMap.dcMotor.get("WL");
        wr.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
        wr.setPower(0);
        wl.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //fr.setMaxSpeed(1200);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //fl.setMaxSpeed(1200);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //br.setMaxSpeed(1200);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //bl.setMaxSpeed(1200);

        wr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motor direction
        fr.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);


        // Define and initialize ALL installed servos.
        push = hwMap.servo.get("push_servo");
        push.setPosition(MID_SERVO);

        beacon = hwMap.colorSensor.get("beacon"); //address 0x3c - default
        beacon.enableLed(false);

        floor = hwMap.colorSensor.get("floor"); //address 0x3a
        floor.setI2cAddress(I2cAddr.create7bit(0x3a));
        floor.enableLed(false);

        touch = hwMap.touchSensor.get("Touch Sensor");
    }

    //  Rotate, Left, Right, Back, Forward
    public final static int Direction_Stop = 0;
    public final static int Direction_Forward = 0b00001;
    public final static int Direction_Reverse = 0b00010;
    public final static int Direction_Left    = 0b01000;
    public final static int Direction_Right   = 0b00100;
    public final static int Direction_Rotate   = 0b10000;
    public final static int Direction_RotateLeft  = Direction_Rotate | Direction_Left;
    public final static int Direction_RotateRight = Direction_Rotate| Direction_Right;
    public final static int Direction_ForwardLeft = Direction_Forward | Direction_Left;
    public final static int Direction_ForwardRight = Direction_Forward | Direction_Right;
    public final static int Direction_ReverseLeft = Direction_Reverse | Direction_Left;
    public final static int Direction_ReverseRight = Direction_Reverse | Direction_Right;

    public void drive(int dir, double pwr){

        switch (dir) {
            case Direction_Stop: motorPower(0,0,0,0); break;
            case Direction_Forward: motorPower(pwr,pwr,pwr,pwr); break;
            case Direction_Reverse: motorPower(-pwr,-pwr,-pwr,-pwr); break;
            case Direction_Right: motorPower(-pwr,pwr,pwr,-pwr); break;
            case Direction_Left: motorPower(pwr,-pwr,-pwr,pwr); break;
            case Direction_RotateRight: motorPower(pwr,-pwr,pwr,-pwr); break;
            case Direction_RotateLeft: motorPower(-pwr,pwr,-pwr,pwr); break;
            case Direction_ForwardLeft: motorPower(pwr,0,0,pwr); break;
            case Direction_ForwardRight: motorPower(0,pwr,pwr,0); break;
            case Direction_ReverseRight: motorPower(-pwr,0,0,-pwr); break;
            case Direction_ReverseLeft: motorPower(0,-pwr,-pwr,0); break;
            default: motorPower(0,0,0,0); break;

        }
    }

    public void stop() {
        motorPower(0,0,0,0);
    }

    public void motorPower(double frpwr, double flpwr, double brpwr, double blpwr) {
        fr.setPower(frpwr);
        fl.setPower(flpwr);
        br.setPower(brpwr);
        bl.setPower(blpwr);
    }


    public boolean isAnyBeaconLight(int threshold) {

        return (beacon.red() > threshold || beacon.blue() > threshold);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

}

