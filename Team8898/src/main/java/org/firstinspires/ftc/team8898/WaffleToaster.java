package org.firstinspires.ftc.team8898;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jack Gonser on 10/9/2017.
 */

@TeleOp(name = "WaffleTeleOp", group = "TeleOp")
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
    private ColorSensor colorSensor = null;

    boolean openRight = false;
    boolean openLeft = false;

    private double servoPos = 0;

    private double drive = 0;
    private double turn = 0;
    private double left = 0;
    private double right = 0;

    public void allServo(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }

    @Override
    public void runOpMode() { // set up
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

        telemetry.addData("Ready", "Go");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }
            if (gamepad1.left_trigger >= 0.1) {
                leftFront.setPower(-left / 4);
                leftBack.setPower(-left / 4);
                rightFront.setPower(-right / 4);
                rightBack.setPower(-right / 4);
            } else {
                leftFront.setPower(-left);
                leftBack.setPower(-left);
                rightFront.setPower(-right);
                rightBack.setPower(-right);
            }
            //Arm servos for grabbing (dpad down)
            if (gamepad1.dpad_down) {
                if (servoPos == 0) {
                    servoPos = 0.8;
                } else {
                    servoPos = 0;
                }
                allServo(servoPos);
                sleep(250);
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
                sleep(500);
                arm.setPower(0);
                allServo(0);
                telemetry.addData("Arm Down", "1/2 second");
            } else if (gamepad1.y) {
                allServo(0.75);
                arm.setPower(0.75);
                sleep(500);
                arm.setPower(0);
                telemetry.addData("Arm Up", "1/2 second"); //add data for the driver station
            } else {
                arm.setPower(.15);
                telemetry.addData("Arm Normal", "Normal Operation");
            }

            //Jewel Servo (always up)
            jewelServo.setPosition(1);

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
                telemetry.addData("Left servo is", "open");
            } else {
                telemetry.addData("Left servo is", "closed");
            }
            if (openRight = true) {
                telemetry.addData("Right servo is", "open");
            } else {
                telemetry.addData("Right servo is", "closed");
            }

            if (jewelServo.getPosition() == 1) {
                telemetry.addData("Jewel Servo is ", "up.");
            } else {
                telemetry.addData("Jewel Servo is", "down.");
            }
            telemetry.update(); //sends data to driver station
        }
    }
}
