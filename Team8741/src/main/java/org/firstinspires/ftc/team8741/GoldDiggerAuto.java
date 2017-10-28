package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Joshua Taufahema on 10/16/2017.
 */
@Autonomous (name = "Gold Digger Auto", group = "Autonomous")
//@Disabled
public class GoldDiggerAuto extends LinearOpMode {
   GoldDiggerBot robot = new GoldDiggerBot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
//use (name).setPower to run motors and Thread.sleep for delay

        //remember to set all setPowers to setTargetPosition
        robot.encoderDrive(28, 28, 0.5);
        //drive to glyph box


        Thread.sleep(1000);

        //use baseball bat to hit glyph into glyph box

        robot.gyroTurn(90, 0.3);
        //turn from glyph box to other team pad

        robot.encoderDrive(38, 38, 0.3);
        //drive to line of middle glyph pit

        robot.gyroTurn(90, 0.3);
        //turn to glyph pit

        robot.encoderDrive(60, 60, 0.6);
        //drive close to glyph pit

        robot.encoderDrive(20, 20, 0.2);
        robot.glyphPull(0.7, 0.7);
        Thread.sleep(1000);
        robot.glyphPull(0, 0);
        //drive through glyph pit and try to grab a glyph

        robot.gyroTurn(180, 0.4);
        //turn back to wall on the south of PC table

        robot.encoderDrive(70, 70, 0.5);
        //drive back to wall

        robot.gyroTurn(90, 0.3);
        //turn back to glyph box

        robot.encoderDrive(48, 48, 0.5);
        //drive to glyph box

        robot.gyroTurn(90, 0.2);
        //turn to be parallel

        Thread.sleep(1500);
        //push last glyph into box, get extra 10 points for being in safe zone


    }
}
