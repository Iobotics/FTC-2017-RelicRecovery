package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import javax.swing.text.html.HTML;

/**
 * Created by Joshua Taufahema on 10/16/2017.
 */
@Autonomous (name = "Gold Digger Auto", group = "Autonomous")
//@Disabled
public class GoldDiggerAuto2 extends LinearOpMode {
    GoldDiggerBot robot = new GoldDiggerBot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
//use (name).setPower to run motors and Thread.sleep for delay
        //remember to set all setPowers to setTargetPosition
        robot.encoderDrive(36, 36, 0.5);
        //drive to glyph box

        robot.gyroTurn(90, 0.3);
        //turn to face parallel to glyph box

        robot.encoderDrive(18, 18, 0.5);
        //drive to glyph box

        robot.baseballBat.setPower(0.8);
        Thread.sleep(1500);
        robot.baseballBat.setPower(0);


    }
}
