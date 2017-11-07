package org.firstinspires.ftc.team8898;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Jack Gonser on 10/9/2017.
 */

@TeleOp (name = "WaffleTeleOp", group = "TeleOp")
//@Disabled
public class WaffleToaster extends LinearOpMode {

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    private DcMotor arm = null;

    private Servo leftServo = null;
    private Servo rightServo = null;
 
    private Servo jewelServo = null;

    long millis = System.currentTimeMillis() % 1000;
    long millisReset;
    boolean openRight= false;
    boolean openLeft = false;

    private double ServoPower = 0;

    private double drive = 0;
    private double turn = 0;
    private double left = 0;
    private double right = 0;

    public void allDrive (double speed) {
        leftBack.setPower(speed);
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
    }
    public void allServo (double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }

    @Override
    public void runOpMode() throws InterruptedException { // set up
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

        telemetry.addData("Ready", "Go");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            millis = System.currentTimeMillis() % 1000;

            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            leftFront.setPower(left);
            leftBack.setPower(left);
            rightFront.setPower(right);
            rightBack.setPower(right);

            //Arm servos for grabbing (dpad down)
            if (gamepad1.dpad_down) {
                if (ServoPower == 0.25) {
                    ServoPower = 0.75;
                } else {
                    ServoPower = 0.25;
                }
                allServo(ServoPower);
                Thread.sleep(250);
            }

            //Arm code (Press a to go up b to go down and y to go up for 1 second)
            if (gamepad1.a) {
                arm.setPower(0.75);
                telemetry.addData("Arm up", "0-1 glyphs"); //add data for the driver station
            } else if (gamepad1.b) {
                arm.setPower(-0.5);
                telemetry.addData("Arm down", ""); //add data for the driver station
            } else if (gamepad1.x) {
                arm.setPower(-0.5);
                Thread.sleep(500);
                arm.setPower(0);
                allServo(0);
                telemetry.addData("Arm Down", "1/2 second");
            } else if (gamepad1.y) {
                allServo(0.75);
                arm.setPower(0.75);
                Thread.sleep(500);
                arm.setPower(0);
                telemetry.addData("Arm Up", "1/2 second"); //add data for the driver station
            } else{
                arm.setPower(.15);
                telemetry.addData("Arm Normal", "Normal Operation");
            }

            //Jewel Servo (always up)
          jewelServo.setPosition(0);

            //telemetry data comparisons
            if (leftServo.getPosition() == 0) {
                openLeft = true;
            } else {
                openLeft = false;
            }
            if (rightServo.getPosition() == 0) {
                openRight = true;
            } else {
                openRight = false;
            }

            if (openLeft = true) {
                telemetry.addData("Left servo is","open");
            } else {
                telemetry.addData("Left servo is", "closed");
            }
            if (openRight = true) {
                telemetry.addData("Right servo is", "open");
            } else {
                telemetry.addData("Right servo is", "closed");
            }

            if (jewelServo.getPosition() == 0) {
                telemetry.addData("Jewel Servo is ", "up.");
            } else {
                telemetry.addData("Jewel Servo is", "down.");
            }
            telemetry.update(); //sends data to driver station
        }
    }
}
