package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftc.vision.BeaconColorResult;
import ftc.vision.FrameGrabber;
import ftc.vision.ImageProcessorResult;
import ftc.vision.JewelColorResult;

/**
 * Created by Joshua Taufahema on 10/16/2017.
 */
@Autonomous (name = "Gold Digger Blue Auto", group = "Linear Opmode")
//@Disabled
public class GoldDiggerBlueAuto extends LinearOpMode {
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

        if (rightColor == JewelColorResult.JewelColor.BLUE){
            robot.jewelServo.setPosition(robot.JEWEL_ARM_DOWN);
            robot.encoderDrive(1, 0.4);
            robot.jewelServo.setPosition(robot.JEWEL_ARM_UP);
            robot.encoderDrive(23, 0.4);

        }
        else if (leftColor == JewelColorResult.JewelColor.BLUE){
            robot.jewelServo.setPosition(robot.JEWEL_ARM_DOWN);
            robot.encoderDrive(-1, -0.4);
            robot.jewelServo.setPosition(robot.JEWEL_ARM_UP);
            robot.encoderDrive(25, 0.4);
        } else {
            robot.encoderDrive(24, 0.4);
        }

        robot.gyroTurn(0.4, -45);

        robot.gyroHold(0.4, -45, 0.5);

        robot.encoderDrive(17, 0.4);

        robot.glyphPull(-0.5, -0.5);
        Thread.sleep(1000);

        robot.glyphPull(0, 0);

        robot.encoderDrive(-5, -0.4);






    }
}
