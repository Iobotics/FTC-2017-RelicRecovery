package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Joshua Taufahema on 10/13/2017.
 */
//Servo = An object designed to give the driver much more control, but less speed than motors
//Motor = An object designed to provide the driver a lot of speed, but less accuracy and precision than Servo
public class GoldDiggerBot {
    //initialises all motors and servos
    public DcMotor leftBackDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftGlyphPull = null;
    public DcMotor rightGlyphPull = null;
    public Servo bottomLift = null;
    public Servo topLift = null;

    public final int TICKS_PER_REV = 560;
    public final int WHEEL_DIAMETER = 4;
    public final double COUNTS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);

    //limits for lift servos
    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        //getting motors
        leftBackDrive = hwMap.get(DcMotor.class, "backLeft");
        leftFrontDrive = hwMap.get(DcMotor.class, "frontLeft");
        rightBackDrive = hwMap.get(DcMotor.class, "backRight");
        rightFrontDrive = hwMap.get(DcMotor.class, "frontRight");
        rightGlyphPull = hwMap.get(DcMotor.class, "rightIntake");
        leftGlyphPull = hwMap.get(DcMotor.class, "leftIntake");
        bottomLift = hwMap.get(Servo.class, "bottomLift");
        topLift = hwMap.get(Servo.class, "topLift");
        //setting direction of motors and how they will spin

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightGlyphPull.setDirection(DcMotor.Direction.FORWARD);
        leftGlyphPull.setDirection(DcMotor.Direction.REVERSE);
        bottomLift.setDirection(Servo.Direction.FORWARD);
        topLift.setDirection(Servo.Direction.FORWARD);

        /* setting motor behaviour */
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //setting the elevator to its minumum position

    }

    public void setMode(DcMotor.RunMode mode){
        leftBackDrive.setMode(mode);
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    public void drive(double leftPower, double rightPower) {
        leftBackDrive.setPower(leftPower);
        leftFrontDrive.setPower(leftPower);
        rightBackDrive.setPower(rightPower);
        rightFrontDrive.setPower(rightPower);
        //sets power motors
    }
    public void stopDrive(){
        drive(0,0);
    }

    public void glyphPull(double leftGlyphPower, double rightGlyphPower) {
        leftGlyphPull.setPower(leftGlyphPower);
        rightGlyphPull.setPower(rightGlyphPower);
        //setspower to the intake
    }

    public void encoderDrive(double inches, double speed) {
        int target = rightBackDrive.getCurrentPosition() + (int) (inches*COUNTS_PER_INCH);
        drive(speed, speed );
        while(rightBackDrive.getCurrentPosition() < target){

        }
        drive(0,0);
    }
}

