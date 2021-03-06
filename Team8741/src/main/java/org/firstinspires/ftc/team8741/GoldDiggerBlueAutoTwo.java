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

@Autonomous (name = "Gold Digger Blue Auto 2", group = "Linear Opmode")
//@Disabled
        public class GoldDiggerBlueAutoTwo extends LinearOpMode {
            GoldDiggerBot robot = new GoldDiggerBot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true);


        robot.encoderDrive(this,30 + robot.knockJewel(JewelColorResult.JewelColor.BLUE),0.4);
        robot.gyroTurn(0.4, 90);
        robot.gyroHold(0.4, 90, 0.5);
        robot.encoderDrive(this,5, 0.4);
        robot.glyphPull(-1);
        sleep(1000);
        robot.glyphPull(0);
        robot.encoderDrive(this,-3, 0.4);
        robot.jewelServo.setPosition(0);
    }

}