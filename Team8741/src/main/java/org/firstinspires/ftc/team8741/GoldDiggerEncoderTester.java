package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by student on 11/10/2017.
 */
@TeleOp(name = "Encoder Tester", group = "TeleOp")
public class GoldDiggerEncoderTester extends LinearOpMode {
    GoldDiggerBot robot = new GoldDiggerBot(this);

    // TANK DRIVE NOT ARCADE
    // Have an else for every if especially
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true);
        telemetry.addLine("Ready To go");
        telemetry.update();
        waitForStart();
        robot.jewelServo.setPosition(robot.JEWEL_ARM_UP);
        while (opModeIsActive()) {
          telemetry.addData("Left Encoder", robot.getLeftEncoder());
          telemetry.addData("Right Encoder", robot.getRightEncoder());
          telemetry.addData("Heading", robot.getGyroHeading());
          telemetry.update();
          if (gamepad1.x){

              robot.jewelServo.setPosition(robot.JEWEL_ARM_UP);
          }
          if (gamepad1.y){

                robot.jewelServo.setPosition(robot.JEWEL_ARM_DOWN);
          }
          if (gamepad1.a){
              robot.displayColors();
          }
            telemetry.addData("Position", robot.jewelServo.getPosition());
        }
    }
}
