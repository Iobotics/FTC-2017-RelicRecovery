package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public Servo rightElevator = null;
    public Servo leftElevator = null;


    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap){

        hwMap = ahwMap;

        //getting motors
        leftBackDrive = hwMap.get(DcMotor.class, "backLeft");
        leftFrontDrive = hwMap.get(DcMotor.class, "frontLeft");
        rightBackDrive = hwMap.get(DcMotor.class, "backRight");
        rightFrontDrive = hwMap.get(DcMotor.class, "frontRight");
        rightGlyphPull = hwMap.get(DcMotor.class, "rightPull");
        leftGlyphPull = hwMap.get(DcMotor.class, "leftPull");
        rightElevator = hwMap.get(Servo.class, "rightLift");
        leftElevator = hwMap.get(Servo.class, "leftLift");
        //setting direction of motors and how they will spin
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightGlyphPull.setDirection(DcMotor.Direction.REVERSE);
        leftGlyphPull.setDirection(DcMotor.Direction.REVERSE);
        rightElevator.setDirection(Servo.Direction.FORWARD);
        leftElevator.setDirection(Servo.Direction.FORWARD);
    }
    public void drive(double leftPower, double rightPower){
        leftBackDrive.setPower(leftPower);
        leftFrontDrive.setPower(leftPower);
        rightBackDrive.setPower(rightPower);
        rightFrontDrive.setPower(rightPower);
    }

    public void glyphPull(double leftGlyphPower, double rightGlyphPower) {
        leftGlyphPull.setPower(leftGlyphPower);
        rightGlyphPull.setPower(rightGlyphPower);
    }
}

