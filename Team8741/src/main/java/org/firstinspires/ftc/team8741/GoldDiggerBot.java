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

    public final int TICKS_PER_REV = 1440;
    public final int WHEEL_DIAMETER = 4;
    public final double COUNTS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);

    public final double MAX_POS = 2.8;
    public final double MIN_POS = 0;
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
        elevatorLift(MIN_POS);

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

    public void elevatorLift(double elevatorPosition) {
        bottomLift.setPosition(elevatorPosition);
        topLift.setPosition(MAX_POS - elevatorPosition);
        //moves elevator position
    }

    public void encoderDrive(double rightInches, double leftInches, double speed) {
        int newLeftTarget;
        int newRightTarget;

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftBackDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = rightBackDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        newRightTarget = rightFrontDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        newLeftTarget = leftFrontDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

        leftBackDrive.setTargetPosition(newLeftTarget);
        leftFrontDrive.setTargetPosition(newLeftTarget);
        rightBackDrive.setTargetPosition(newRightTarget);
        rightFrontDrive.setTargetPosition(newLeftTarget);

        // Turn On RUN_TO_POSITION
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive(speed, speed);

        while ((leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

        }

        // Stop all motion;
        drive(0, 0);

        // Turn off RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

