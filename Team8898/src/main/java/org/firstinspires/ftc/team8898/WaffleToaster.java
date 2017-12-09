package org.firstinspires.ftc.team8898;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.SwitchableLight;

/**
 * Created by Jack Gonser on 10/9/2017.
 * yay
 * hi
 * I SAID HEY HEY HEY HEY HEY HEY AHA WHATS GOING ON? -heman
 * tyler is a person :0
 * FAT SANTA BOIIIISSSS
 * POSITIVE VIBES FOR LIVE - said no one
 * The meme dream is alive
 */

@TeleOp(name = "WaffleTeleOp", group = "TeleOp")
//@Disabled
public class WaffleToaster extends LinearOpMode {
    WaffleToasterMain robot = new WaffleToasterMain();

    boolean openRight = false;
    boolean openLeft = false;

    private double servoPos = 0;

    private double drive = 0;
    private double turn = 0;
    private double left = 0;
    private double right = 0;
    // private double strafe

    @Override
    public void runOpMode() { // set up
        robot.init(hardwareMap,false);

        telemetry.addData("Ready", "Go");
        telemetry.update();

        waitForStart(); // wait for activation on Driver station

        while (opModeIsActive()) { // While driver station is set to run
            /*
            TODO: Add butterfly drive
             *driving controls
             */
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            //turn = gamepad1.left_stick_x;
            // strafe = gamepad1.right_stick_x

            // Combine drive and turn for blended motion.
            left = drive + turn;
            right = drive - turn;
            /*
            *replace above with this
            if(turn > 0 || turn < 0) {
                robot.allDrive(turn,turn);
            }
            if(drive > 0 || drive < 0) {
                robot.rightFront.setPower(drive);
                robot.leftBack.setPower(-drive);
            }

             */
            // Normalize the values so neither exceed +/- 1.0
            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }
            if (gamepad1.left_trigger >= 0.1) {
                left = left/4;
                right = right/4;
                robot.allDrive(left,right);
            } else {
                robot.allDrive(left, right);
            }
            //Arm servos for grabbing (dpad down)
            if (gamepad1.dpad_down) {
                if (servoPos == 0) {
                    servoPos = 0.8;
                } else {
                    servoPos = 0;
                }
                robot.allServo(servoPos);
                sleep(250);
            }

            //Arm code (Press a to go up b to go down and y to go up for 1 second)
            if (gamepad1.a) {
                robot.arm.setPower(0.75);
                telemetry.addData("Arm up", "User Controlled"); //add data for the driver station
            } else if (gamepad1.b) {
                robot.arm.setPower(-0.5);
                telemetry.addData("Arm down.", "User Controlled"); //add data for the driver station
            } else if (gamepad1.x) {
                robot.arm.setPower(-0.5);
                sleep(500);
                robot.arm.setPower(0.2);
                sleep(200);
                robot.allServo(0);
                telemetry.addData("Arm Down", "1/2 second.");
            } else if (gamepad1.y) {
                robot.allServo(0.75);
                robot.arm.setPower(0.75);
                sleep(500);
                robot.arm.setPower(0);
                telemetry.addData("Arm Up", "1/2 second."); //add data for the driver station
            } else if (gamepad1.right_stick_button) {
                telemetry.addData("Current Value:", robot.arm.getCurrentPosition());
            } else {
                robot.arm.setPower(.15);
                telemetry.addData("Arm Normal", "Normal Operation :)");
            }

            //Jewel Servo (always up)
            robot.jewelServo.setPosition(1);

            //telemetry data comparisons
            if (robot.leftServo.getPosition() == 0) {
                openLeft = true;
            } else {
                openLeft = false;
            }
            if (robot.rightServo.getPosition() == 0) {
                openRight = true;
            } else {
                openRight = false;
            }

            if (openLeft = true) {
                telemetry.addData("Left servo is", "open.");
            } else {
                telemetry.addData("Left servo is", "closed.");
            }
            if (openRight = true) {
                telemetry.addData("Right servo is", "open.");
            } else {
                telemetry.addData("Right servo is", "closed.");
            }

            if (robot.jewelServo.getPosition() == 1) {
                telemetry.addData("Jewel Servo is", "up.");
            } else {
                telemetry.addData("Jewel Servo is", "down.");
            }
            if (robot.colorSensor instanceof SwitchableLight) {
                ((SwitchableLight)robot.colorSensor).enableLight(false);
            }
            telemetry.update(); //sends data to driver station
        }
    }
}
