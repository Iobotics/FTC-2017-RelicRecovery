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
@Autonomous (name = "Gold Digger Blue Auto 1", group = "Linear Opmode")
//@Disabled
public class GoldDiggerBlueAuto extends LinearOpMode {
    GoldDiggerBot robot = new GoldDiggerBot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true);
        waitForStart();


            if (false){
                robot.jewelServo.setPosition(robot.JEWEL_ARM_DOWN);
                robot.encoderDrive(1, 0.4);
                robot.jewelServo.setPosition(robot.JEWEL_ARM_UP);
                robot.encoderDrive(23, 0.4);

            }
            else if (false){
                robot.jewelServo.setPosition(robot.JEWEL_ARM_DOWN);
                robot.encoderDrive(-1, 0.4);
                robot.jewelServo.setPosition(robot.JEWEL_ARM_UP);
                robot.encoderDrive(25, 0.4);
            } else {
          //drives forward 24 inches no jewels are found
            robot.encoderDrive(24, 0.4);
       }
        //turns 45 degrees clockwise to line up with cryptobox
        //robot.gyroTurn(0.4, -45);
        robot.gyroHold(0.4, -45, 2);

        //drives forward to meet box
        robot.encoderDrive(9, 0.4);

        //outtakes Glyph
        robot.glyphPull(-1);
        sleep(1000);
        robot.glyphPull(0);

        //drives back out of zone
        robot.encoderDrive(-11, 0.4);

        //Turns another 45 degrees to line up with the safe zone
        //robot.gyroTurn(0.4, -90);
        robot.gyroHold(0.4, -90, 2);

        //drives back into zone
        robot.encoderDrive(9, 0.4);

    }
}
