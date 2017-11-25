package org.firstinspires.ftc.team8898;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import static java.lang.System.currentTimeMillis;

/**
 * Created by Matt Hockenberger and Jack Gonser on 10/28/2017.
 */
@Autonomous (name = "AUTO(BLUE)(Audience side)",group = "BLUE")
public class WaffleToasterautocompBLUE extends LinearOpMode {
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
    NormalizedColorSensor colorSensor;
    View relativeLayout;
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

        if (leftPower == 0 && rightPower == 0) {
            telemetry.addData("Robot is stopped.", "STOP");
        } else if (leftPower > 0 && rightPower > 0) {
            telemetry.addData("Robot is going forward.", "FORWARD");
        } else if (leftPower < 0 && rightPower < 0) {
            telemetry.addData("Robot is going backward.","BACKWARD");
        } else {
            telemetry.addData("Robot is operating abnormally or is turning.", "OTHER");
        }
        telemetry.update();
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
            telemetry.addData("Turning left by", speed + "power.");
        } else if (direction == "right") {
            leftFront.setPower(speed);
            leftBack.setPower(speed);
            rightFront.setPower(-speed);
            rightBack.setPower(-speed);
            telemetry.addData("Turning right by", speed + "power.");
        } else {
            telemetry.addData("Uh Oh", "Incorrect syntax");
        }
        telemetry.update();
    }

    public void runOpMode() { //set up for the phone tp
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        arm = hardwareMap.get(DcMotor.class, "arm");

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        jewelServo = hardwareMap.get(Servo.class, "jewelServo");

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);



        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);

        jewelServo.setDirection(Servo.Direction.FORWARD);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        waitForStart();

        leftServo.setPosition(0.25);
        rightServo.setPosition(0.25);
        jewelServo.setPosition(.4);
        arm.setPower(0.25);
        sleep(500);
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        int color = colors.toColor();
        long startTime = currentTimeMillis();
        while(Color.red(color) <= 3 && currentTimeMillis()- startTime < 500) {
            colors = colorSensor.getNormalizedColors();
            color = colors.toColor();
            allDrive(0,0);
        }
        allDrive(0,0);
        sleep(100);
        int timeDiff;
        telemetry.addData("Red", Color.red(color));
        if(Color.red(color) > 3){
            allDrive(.4, .4);
            timeDiff = -500;
        }
        else{
            allDrive(-.4,-.4);
            timeDiff = 550;
        }
        sleep(400);

        allDrive(0,0);
        jewelServo.setPosition(1);
        sleep(200);
        allDrive(0.4,0.4);
        sleep(1300-timeDiff);
        turnDrive("right", 0.4);
        sleep(300);
        allDrive(0,0);
        allServo(0);
    }
}
