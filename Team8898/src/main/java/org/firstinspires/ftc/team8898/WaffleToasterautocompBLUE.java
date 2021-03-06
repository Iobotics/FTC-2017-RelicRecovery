package org.firstinspires.ftc.team8898;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import static java.lang.System.currentTimeMillis;

/**
 * Created by Matt Hockenberger and Jack Gonser on 10/28/2017.
 */
@Autonomous (name = "AUTO(BLUE)(Audience side)",group = "BLUE")
public class WaffleToasterautocompBLUE extends LinearOpMode {
    View relativeLayout;
    WaffleToasterMain robot = new WaffleToasterMain();
    private boolean jewelIsBlue;

    public void runOpMode() { //set up for the phone tp
        robot.init(hardwareMap, true);
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (robot.colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) robot.colorSensor).enableLight(true);
        }

        waitForStart();

        robot.allServo(0.25);
        robot.jewelServo.setPosition(.4);
        robot.arm.setPower(0.25);
        sleep(500);
        NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();
        int color = colors.toColor();
        long startTime = currentTimeMillis();
        while (Color.red(color) <= 3 && currentTimeMillis() - startTime < 500) {
            colors = robot.colorSensor.getNormalizedColors();
            color = colors.toColor();
            robot.allDrive(0, 0);
        }
        robot.allDrive(0, 0);
        sleep(100);
        if (Color.red(color) < 3) {
            robot.encoderDrive(this, -0.5, 0.4);
            jewelIsBlue = true;
        } else {
            robot.encoderDrive(this,0.5, 0.4);
            jewelIsBlue = false;
        }
        sleep(1000);
        robot.resetRobot("jewel");
        if (jewelIsBlue) {
            robot.encoderDrive(this,-32.5, 0.4);
        } else {
            robot.encoderDrive(this,-33.5, 0.4);
        }
    }
}
