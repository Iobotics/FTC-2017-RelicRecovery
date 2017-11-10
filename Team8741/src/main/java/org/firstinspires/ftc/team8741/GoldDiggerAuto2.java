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
@Autonomous (name = "Gold Digger Auto 2jj", group = "Linear Opmode")
//@Disabled
public class GoldDiggerAuto2 extends LinearOpMode {
    GoldDiggerBot robot = new GoldDiggerBot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true);
        waitForStart();

        robot.gyroDrive(0.5, 36, 0);
        //go forward

        robot.gyroTurn(0.4, 90);
        //turn towards the crypto box

        robot.gyroHold(0.4, 90, 0.5);
        //gives robot time to reset

        robot.gyroDrive(0.5, 10, 0);
        //go up to crypto box

        robot.glyphPull(-1,-1);
        Thread.sleep(1500);
        //put pre-loaded glyph into the crypto box

        robot.glyphPull(0,0);
        //turn off glyph motors

        //THIS IS FOR THE STAND NEAREST TO BOTH CRYPTO BOXES AND PUTS ON GLYPH AND GETS INTO THE TRIANGLE OF POINTS

    }

}