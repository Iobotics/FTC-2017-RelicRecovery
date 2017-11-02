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
   GoldDiggerBot robot = new GoldDiggerBot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        robot.encoderDrive(32,0.4);
    }
}
