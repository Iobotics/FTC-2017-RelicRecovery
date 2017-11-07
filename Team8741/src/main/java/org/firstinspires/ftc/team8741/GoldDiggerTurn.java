package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by student on 11/3/2017.
 */
@Autonomous (name = "Gold Digger Turn", group = "Linear Opmode")

public class GoldDiggerTurn extends LinearOpMode{
    GoldDiggerBot robot = new GoldDiggerBot(this);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true);
        waitForStart();
        robot.drive(0.3, 0.3);
        Thread.sleep(500);
        robot.stopDrive();
    }
}
