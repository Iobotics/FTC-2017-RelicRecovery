package org.firstinspires.ftc.team8898;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftc.vision.BeaconColorResult;
import ftc.vision.FrameGrabber;
import ftc.vision.ImageProcessorResult;

/**
 * Created by Jack Gonser on 10/16/2017.
*/
@Autonomous(name = "WaffleToasterAutoRED", group = "Auto")
@Disabled
public class WaffleToasterAuto extends LinearOpMode {
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    private DcMotor arm = null;

    private Servo leftServo = null;
    private Servo rightServo = null;

    private Servo jewelServo = null;
    private ColorSensor colorSensor = null;

    private BeaconColorResult.BeaconColor leftColorCheck = null;
    private BeaconColorResult.BeaconColor rightColorCheck = null;
    //sets basic variables to set servos and motors to
    private float stop = 0;
    private float forward = 1;
    private float backward = -1;
    private float halfBack = -0.5f;
    private float halfForward = 0.5f;

    long millis = System.currentTimeMillis() % 1000;
    long millisReset;
    boolean openRight = false;
    boolean openLeft = false;
    public int secondsToMillis(double seconds) {return (int) seconds * 1000;}

    public void allDrive(float speed) {
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
    }

    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        arm = hardwareMap.get(DcMotor.class, "arm");

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        jewelServo = hardwareMap.get(Servo.class, "jewelServo");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);

        jewelServo.setDirection(Servo.Direction.FORWARD);


        waitForStart();


        colorSensor.enableLed(true); //activate Led on color sensor
        jewelServo.setPosition(stop); //drop jewel arm
        sleep(500);
        if (colorSensor.blue() >= 128) { //Sense
            telemetry.addData("Going for the jewel on the left.", "Blue Jewel is on the right");
            telemetry.addData("Jewel Arm Down", "");
            telemetry.update();
            leftServo.setPosition(0.25);
            rightServo.setPosition(0.25);
            arm.setPower(0.25);
            Thread.sleep(secondsToMillis(0.5));
            arm.setPower(stop);
            allDrive(halfBack);
            Thread.sleep(secondsToMillis(0.25));
            jewelServo.setPosition(stop);
            Thread.sleep(secondsToMillis(0.25));
            allDrive(halfForward);
            Thread.sleep(secondsToMillis(2.5));
            allDrive(stop);
        } else if (colorSensor.blue() < 128) { //senses if the blue jewel is on the left
            telemetry.addData("Going for the jewel on the left.", "Blue Jewel is on the left");//adds data to package to send towards the driver station
            telemetry.update();//sends package to the driver station
            leftServo.setPosition(0.25);
            rightServo.setPosition(0.25);
            arm.setPower(0.25);
            Thread.sleep(secondsToMillis(0.5));
            arm.setPower(stop);
            allDrive(halfForward);
            jewelServo.setPosition(stop);
            Thread.sleep(secondsToMillis(2));
            allDrive(stop);
        }
        telemetry.update();
    }

}

