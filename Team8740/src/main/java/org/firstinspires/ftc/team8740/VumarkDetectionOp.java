package org.firstinspires.ftc.team8740;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by darrenk1801 on 11/25/17.
 */
@TeleOp(name = "Team 8740: Test Vuforia", group = "Team 8740")
@Disabled
public class VumarkDetectionOp extends LinearOpMode {

    Team8740_Base robot = new Team8740_Base();
    RelicRecoveryVuMark vuMark = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.setHardwareMap(hardwareMap);
        robot.setOpmode(this);

        robot.initVuforia();
        robot.activateVuforia();

        while (!isStarted()) {
            vuMark = robot.getVuMark();

            telemetry.addData("VuMark", vuMark); //Display it on telemetry
            telemetry.update();

            sleep(100);
        }
    }
}
