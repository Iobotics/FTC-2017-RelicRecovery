package org.firstinspires.ftc.team8898;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import ftc.vision.BeaconColorResult;

import static java.lang.System.currentTimeMillis;

/**
 * Created by Teacher on 10/28/2017.
 */
@Autonomous (name = "AUTO(BLUE)(if auto fails)",group = "BLUE")
public class WaffleToasterautocompBLUE extends WaffleToasterAuto {
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


    public void runOpMode() throws InterruptedException { //set up for the phone tp
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



        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

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
        Thread.sleep(500);
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        int color = colors.toColor();
        long startTime = currentTimeMillis();
        while(Color.red(color) <= 3 && currentTimeMillis()- startTime < 500) {
            colors = colorSensor.getNormalizedColors();
            color = colors.toColor();
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        Thread.sleep(100);
        int timeDiff;
        telemetry.addData("Red", Color.red(color));
        if(Color.red(color) > 3){
            leftFront.setPower(-.4);
            leftBack.setPower(-.4);
            rightFront.setPower(-.4);
            rightBack.setPower(-.4);
            timeDiff = 500;
        }
        else{
            leftFront.setPower(.4);
            leftBack.setPower(.4);
            rightFront.setPower(.4);
            rightBack.setPower(.4);
            timeDiff = -350;
        }
        Thread.sleep(350);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        jewelServo.setPosition(1);
        Thread.sleep(200);

        leftFront.setPower(-.4);
        leftBack.setPower(-.4);
        rightFront.setPower(-.4);
        rightBack.setPower(-.4);
        Thread.sleep(1750-timeDiff);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
