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
@Autonomous (name = "Gold Digger Auto", group = "Linear Opmode")
//@Disabled
public class GoldDiggerAuto extends LinearOpMode {
    GoldDiggerBot robot = new GoldDiggerBot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true);
        waitForStart();

        robot.gyroDrive(0.5, 25, 0);
        //goes forward

        robot.gyroTurn(0.4, 90);
        //turns to be parallel to the glyph box

        robot.gyroDrive(0.5, 32, 0);
        //goes into glyph box or at least go up to it

        robot.glyphPull(-1, -1);
        Thread.sleep(1500);
        //put preloaded glyph into the crypto box

        robot.glyphPull(0, 0);
        //turn off glyph pull motors

        //THIS IS THE AUTO FOR THE BLUE NEXT TO ONLY ONE CRYPTO BOX AND GETS INTO THE TRIANGLE OF POINTS

    }
}
