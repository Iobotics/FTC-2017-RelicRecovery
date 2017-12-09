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
@Autonomous (name = "AUTO(RED)(Back Glyph Box)",group = "RED")
public class WaffleToasterAuto extends LinearOpMode {
    WaffleToasterMain robot = new WaffleToasterMain();
    View relativeLayout;
    boolean isBlue;


    public void runOpMode()  { //set up for the phone tp
        robot.init(hardwareMap, false);
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (robot.colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)robot.colorSensor).enableLight(true);
        }

        waitForStart();
        robot.initGyro();
        robot.allServo(0.25);
        robot.jewelServo.setPosition(0.4);
        robot.arm.setPower(0.25);
        sleep(500);
        NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();
        int color = colors.toColor();
        long startTime = currentTimeMillis();
        while(Color.red(color) < 3 && currentTimeMillis()- startTime < 900) {
            colors = robot.colorSensor.getNormalizedColors();
            color = colors.toColor();
            robot.allDrive(0, 0);
        }
        robot.allDrive(0, 0);
        sleep(100);
        if(Color.red(color) < 3){
            robot.encoderDrive(this,-1,0.4);
            isBlue = false;
        }
        else{
            robot.encoderDrive(this,1,0.4);
            isBlue = true;
        }
        sleep(500);

        robot.resetRobot("all");
        sleep(1000);
        if (isBlue) {
            robot.encoderDrive(this,15,0.4);
        } else {
            robot.encoderDrive(this,17,0.4);
        }
        robot.gyroTurn(0.4,90);
        robot.encoderDrive(this,13,0.4);
        robot.gyroTurn(0.4,-90);
        robot.encoderDrive(this,2.4,0.4);
        robot.resetRobot("drive");
    }
}
