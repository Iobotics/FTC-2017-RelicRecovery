package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Joshua Taufahema on 11/8/2017.
 */
@Autonomous (name = "Gold Digger Auto", group = "Linear Opmode")
//@Disabled
public class GoldDiggerAuto2 extends LinearOpMode {
    GoldDiggerBot robot = new GoldDiggerBot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true);
        waitForStart();

        robot.gyroDrive(0.3, 36, 0);
        //go forward

        robot.gyroTurn(0.2, 90);
        //turn towards the crypto box

        robot.encoderDrive(10, 0.3);
        //go up to crypto box

        robot.glyphPull(-0.5, -0.5);
        Thread.sleep(1500);
        //put pre-loaded glyph into the crypto box

        robot.glyphPull(0, 0);
        //turn off glyph motors


        //THIS IS FOR THE STAND NEAREST TO BOTH CRYPTO BOXES AND PUTS ON GLYPH AND GETS INTO THE TRIANGLE OF POINTS




    }

}