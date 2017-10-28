package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Joshua Taufahema on 10/13/2017.
 */
//Servo = An object designed to give the driver much more control, but less speed than motors
//Motor = An object designed to provide the driver a lot of speed, but less accuracy and precision than Servo
public class GoldDiggerBot {
    public DcMotor leftBackDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftGlyphPull = null;
    public DcMotor rightGlyphPull = null;
    public Servo bottomLift = null;
    public Servo topLift = null;
    public GyroSensor gyro = null;


    public final int TICKS_PER_REV = 1440;
    public final int WHEEL_DIAMETER = 4;
    public final double COUNTS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);
    public final double MAX_POS = 1.0;
    public final double MIN_POS = 0;
    public double position = 0;
    private int degreesReset;

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

        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");

        //setting direction of motors and how they will spin
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightGlyphPull.setDirection(DcMotor.Direction.FORWARD);
        leftGlyphPull.setDirection(DcMotor.Direction.REVERSE);
        bottomLift.setDirection(Servo.Direction.REVERSE);
        topLift.setDirection(Servo.Direction.FORWARD);
        elevatorLift(MIN_POS);
    }

    public void drive(double leftPower, double rightPower) {
        leftBackDrive.setPower(leftPower);
        leftFrontDrive.setPower(leftPower);
        rightBackDrive.setPower(rightPower);
        rightFrontDrive.setPower(rightPower);
    }

    public void glyphPull(double leftGlyphPower, double rightGlyphPower) {
        leftGlyphPull.setPower(leftGlyphPower);
        rightGlyphPull.setPower(rightGlyphPower);
    }

    public void elevatorLift(double elevatorPosition) {
        bottomLift.setPosition(elevatorPosition);
        bottomLift.setPosition(elevatorPosition);
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

    public void gyroTurn(double degrees, double speed) {
        if(degrees > 0){
            while(degrees != degreesReset - gyro.getHeading() )
            leftBackDrive.setPower(-1);
        }
        else{
            leftBackDrive.setPower(-1);
        }
    }
}

