package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Joshua Taufahema on 10/16/2017.
 */
@Autonomous (name = "GoldDiggerAuto", group = "Autonomous")
//@Disabled
public class GoldDiggerAuto extends LinearOpMode {
   GoldDiggerBot robot = new GoldDiggerBot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
//use (name).setPower to run motors and Thread.sleep for delay
        robot.leftFrontDrive.setPower(.5);
        robot.leftBackDrive.setPower(.5);
        robot.rightFrontDrive.setPower(.5);
        robot.rightBackDrive.setPower(.5);
        Thread.sleep(1500);
        robot.leftFrontDrive.setPower(-.3);
        robot.leftBackDrive.setPower(-.3);
        robot.rightFrontDrive.setPower(.3);
        robot.rightBackDrive.setPower(.3);
        Thread.sleep(1000);
        robot.leftFrontDrive.setPower(.4);
        robot.leftBackDrive.setPower(.4);
        robot.rightFrontDrive.setPower(.4);
        robot.rightBackDrive.setPower(.4);
        robot.rightGlyphPull.setPower(.5);
        robot.leftGlyphPull.setPower(.5);
        Thread.sleep(3000);




    }
}
