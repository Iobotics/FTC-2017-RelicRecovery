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

        robot.encoderDrive(this, -(30+robot.knockJewel(JewelColorResult.JewelColor.RED)), 0.4);

        robot.gyroHold(0.4, 90, 2);
        robot.encoderDrive(this,5, 0.4);
        robot.glyphPull(-1);
        sleep(1000);
        robot.glyphPull(0);
        robot.encoderDrive(this,-3, 0.4);
    }

}