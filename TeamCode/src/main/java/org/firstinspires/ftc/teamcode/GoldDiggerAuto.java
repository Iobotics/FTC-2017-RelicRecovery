package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Joshua Taufahema on 10/16/2017.
 */
@Autonomous (name = "GoldDiggerAuto", group = "Autonomous")
public class GoldDiggerAuto extends LinearOpMode {
    public DcMotor leftBackDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftGlyphPull = null;
    public DcMotor rightGlyphPull = null;
    public Servo leftElevator = null;
    public Servo rightElevator = null;
    @Override
    public void runOpMode() throws InterruptedException {

    }
}
