package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftc.vision.FrameGrabber;
import ftc.vision.ImageProcessorResult;
import ftc.vision.JewelColorResult;

/**
 * Created by Joshua Taufahema on 11/8/2017.
 */

@Autonomous (name = "Gold Digger Red Auto 2", group = "Linear OpMode")
//Disabled
public class GoldDiggerRedAutoTwo extends LinearOpMode {
    GoldDiggerBot robot = new GoldDiggerBot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true);
        waitForStart();
        FrameGrabber frameGrabber = FtcRobotControllerActivity.frameGrabber; //Get the frameGrabber

        frameGrabber.grabSingleFrame(); //Tell it to grab a frame
        while (!frameGrabber.isResultReady()) { //Wait for the result
            Thread.sleep(5); //sleep for 5 milliseconds
        }

//Get the result
        ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
        JewelColorResult result = (JewelColorResult) imageProcessorResult.getResult();

        JewelColorResult.JewelColor leftColor = result.getLeftColor();
        JewelColorResult.JewelColor rightColor = result.getRightColor();

        telemetry.addData("Result", result); //Display it on telemetry
        telemetry.update();


//wait before quitting (quitting clears telemetry)
        Thread.sleep(1000);

        robot.encoderDrive(30, 0.4);
        robot.gyroTurn(0.4, -90);
        robot.gyroHold(0.4, -90, 0.5);
        robot.encoderDrive(10, 0.4);
        robot.glyphPull(-1);
        Thread.sleep(1000);
        robot.glyphPull(0);
        robot.encoderDrive(-7, 0.4);
    }

}