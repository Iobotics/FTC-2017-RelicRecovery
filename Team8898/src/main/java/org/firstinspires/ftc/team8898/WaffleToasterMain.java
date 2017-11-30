package org.firstinspires.ftc.team8898;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

/**
 * Created by Jack Gonser :) on 11/27/2017.
 */

public class WaffleToasterMain {
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    // creates arm variable
    public DcMotor arm = null;
    // sets up variable for the arm servos
    public Servo leftServo = null;
    public Servo rightServo = null;
    // sets up variable for the jewel servo
    public Servo jewelServo = null;
    NormalizedColorSensor colorSensor;

    HardwareMap hwmap = null;

    private boolean teleOp;
    // Thanks Joel
    private final int TICKS_PER_REV = 1120;
    private final int WHEEL_DIAMETER = 4;
    private final double COUNTS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);

    public void WaffleToasterMain (boolean teleOpCheck) {
        if (teleOpCheck) {
            teleOp = true;
        } else {
            teleOp = false;
        }
    }
    void init (HardwareMap awhmap, boolean autoBlue) {
        hwmap = awhmap;

        leftFront = hwmap.get(DcMotor.class, "leftFront");
        leftBack = hwmap.get(DcMotor.class, "leftBack");
        rightFront = hwmap.get(DcMotor.class, "rightFront");
        rightBack = hwmap.get(DcMotor.class, "rightBack");

        arm = hwmap.get(DcMotor.class, "arm");

        leftServo = hwmap.get(Servo.class, "leftServo");
        rightServo = hwmap.get(Servo.class, "rightServo");

        jewelServo = hwmap.get(Servo.class, "jewelServo");

        colorSensor = hwmap.get(NormalizedColorSensor.class, "colorSensor");

        if (autoBlue && !teleOp) {
            leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
            leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        } else if (teleOp) {
            //for butterfly drive
            leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
            leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        }
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);

        jewelServo.setDirection(Servo.Direction.FORWARD);
    }

    /**
     * Sleep from LinearOpMode
     * @param milliseconds
     */
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Sets the two arm servos to a set position
     * @param position
     */
    public void allServo(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }

    /**
     * Sets power to the four motors by left or right values
     * @param leftPower
     * @param rightPower
     */
    public void allDrive (double leftPower, double rightPower) {
        leftBack.setPower(leftPower);
        leftFront.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);

    }

    /**
     * Turns the robot depending on the direction you set it to turn and the power you set it to go
     * @param direction
     * @param speed
     */
    public void turnDrive (String direction, double speed) {
        direction = direction.toLowerCase();
        if (direction == "left") {
            leftFront.setPower(-speed);
            leftBack.setPower(-speed);
            rightFront.setPower(speed);
            rightBack.setPower(speed);
        } else if (direction == "right") {
            leftFront.setPower(speed);
            leftBack.setPower(speed);
            rightFront.setPower(-speed);
            rightBack.setPower(-speed);
        } else {
        }
    }

    /**
     * Resets robot to what you put in
     * drive stops the robot
     * arm lowers and opens the servos
     * jewel brings up the arm and turns the light off
     * all resets both the jewel and the drive motors
     * @param resetParts
     */
    public void resetRobot (String resetParts) {
        resetParts = resetParts.toLowerCase();
        if (resetParts == "drive") {
            allDrive(0,0);
        } else if (resetParts == "jewel") {
            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight)colorSensor).enableLight(false);
            }
            jewelServo.setPosition(1);
        } else if (resetParts == "all") {
            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight)colorSensor).enableLight(false);
            }
            jewelServo.setPosition(1);
            allDrive(0,0);
        } else if (resetParts == "arm") {
            allServo(0);
            arm.setPower(0.4);
            sleep(200);
            arm.setPower(0);
        }
    }

    /**
     * Sets the speed for strafing and normal drive with a butterfly drive train
     * @param speed
     * @param forward
     */
    public void twoMotorDrive (double speed, boolean forward) {
        if (forward) {
            leftFront.setPower(speed);
            rightBack.setPower(-speed);
        } else {
            leftBack.setPower(-speed);
            rightFront.setPower(speed);
        }
    }
    //Thanks Joel
    public void encoderDrive(double inches, double speed) {
        //sets the target encoder value
        int target = rightFront.getCurrentPosition() + (int) (inches*COUNTS_PER_INCH);
        //if positive move forward to meet the value
        if (inches > 0) {
            allDrive(speed, speed);
            while (rightFront.getCurrentPosition() <= target) {
                //pauses the robot while moving forward
            }
            resetRobot("drive");
        }
        //if negative move backwards
        else if (inches < 0){
            allDrive(-speed, -speed);
            while (rightFront.getCurrentPosition() >= target) {
                //pauses the robot while moving back
            }
            resetRobot("drive");
        }
    }
}
