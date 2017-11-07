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
    //initialises all motors and servos
    private DcMotor leftBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftGlyphPull = null;
    private DcMotor rightGlyphPull = null;

    private final int TICKS_PER_REV = 560;
    private final int WHEEL_DIAMETER = 4;
    private final double COUNTS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);

    private HardwareMap hwMap = null;

    void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        //getting motors
        leftBackDrive = hwMap.get(DcMotor.class, "backLeft");
        leftFrontDrive = hwMap.get(DcMotor.class, "frontLeft");
        rightBackDrive = hwMap.get(DcMotor.class, "backRight");
        rightFrontDrive = hwMap.get(DcMotor.class, "frontRight");
        rightGlyphPull = hwMap.get(DcMotor.class, "rightIntake");
        leftGlyphPull = hwMap.get(DcMotor.class, "leftIntake");


        //setting direction of motors and how they will spin

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightGlyphPull.setDirection(DcMotor.Direction.FORWARD);
        leftGlyphPull.setDirection(DcMotor.Direction.REVERSE);

        /* setting motor behaviour */
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //setting the elevator to its minimum position
    }

    public void setMode(DcMotor.RunMode mode){
        leftBackDrive.setMode(mode);
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        rightBackDrive.setMode(mode);
        //set run mode of the motors
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
        //stops the robot
    }

    public void glyphPull(double leftGlyphPower, double rightGlyphPower) {
        leftGlyphPull.setPower(leftGlyphPower);
        rightGlyphPull.setPower(rightGlyphPower);
        //sets power to the intake
    }
    /*
    *Autonomously moves the bot forward a set number of inches
    *@param inches sets the amount of inches the robot goes forward
    * @param speed sets the speed the robot moves at
     */
    public void encoderDrive(double inches, double speed) {
        int target = rightBackDrive.getCurrentPosition() + (int) (inches*COUNTS_PER_INCH);
        drive(speed, speed );
        while(rightBackDrive.getCurrentPosition() < target){

        }
        stopDrive();
    }

    public void gyroTurn(int degrees, double speed){

    }
}

