package org.firstinspires.ftc.team8898;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import ftc.vision.BeaconColorResult;

/**
 * Created by Teacher on 10/28/2017.

@Autonomous (name = "AUTO(allcolor)(if auto fails)",group = "RED")
public class WaffleToasterautocompRED extends WaffleToasterAuto {
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    // creates arm variable
    private DcMotor arm = null;
    // sets up variable for the arm servos
    private Servo leftServo = null;
    private Servo rightServo = null;
    // sets up variable for the jewel servo
    private Servo jewelServo = null;


    public void runOpMode() throws InterruptedException { //set up for the phone tp
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        arm = hardwareMap.get(DcMotor.class, "arm");

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        jewelServo = hardwareMap.get(Servo.class, "jewelServo");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);

        jewelServo.setDirection(Servo.Direction.FORWARD);

        waitForStart();
        leftServo.setPosition(0.25);
        rightServo.setPosition(0.25);
        arm.setPower(0.25);
        Thread.sleep(500);
        allDrive(1);
        Thread.sleep(2000);
        allDrive(0);
    }
}
*/