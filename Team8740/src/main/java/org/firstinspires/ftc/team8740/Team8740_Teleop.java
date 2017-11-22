/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.team8740;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Team 8740: Teleop Mecanum", group="Team 8740")
//@Disabled
public class Team8740_Teleop extends LinearOpMode {

    Team8740_Base robot = new Team8740_Base();

    @Override
    public void runOpMode() {

        //Initialize the hardware variables.
        robot.initRobot(hardwareMap, this, true);

        // Send telemetry message to signify robot waiting
        telemetry.addData("O", "Waiting for start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        gamepad1.setJoystickDeadzone((float) 0.05);
        gamepad2.setJoystickDeadzone((float) 0.05);

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /* GAMEPAD 1 CONTROLS */

            // Mecanum drive uses left stick to strafe along X and Y, and right stick X to turn.
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;

            double rotation = gamepad1.right_stick_x;

            robot.setMecanum(x, y, rotation);

            // Use gamepad left bumper to toggle speed
            if(gamepad1.left_bumper) {
                robot.toggleSpeed();
            }

            // Use gamepad right bumper to run intake
            if(gamepad1.right_bumper) {
                robot.runIntake();
            } else if(gamepad1.right_trigger > 0.2) {
                robot.reverseIntake();
            } else {
                robot.stopIntake();
            }

            // Use gamepad A to toggle top outtake
            if(gamepad1.a) {
                robot.pushGlyph();
                robot.toggleIntakeClaws();
                sleep(1430);
                robot.retractGlyph();
                sleep(1430);
                robot.stopGlyph();
            } else {
                robot.stopGlyph();
            }

            // Use gamepad B to toggle all outtakes
            if(gamepad1.b) {
                robot.reverseIntake();
                robot.pushGlyph();
                robot.toggleIntakeClaws();
                sleep(1430);
                robot.stopIntake();
                robot.retractGlyph();
                sleep(1430);
                robot.stopGlyph();
            } else {
                robot.stopGlyph();
            }

            // Use gamepad X to toggle bottom outtake
            if(gamepad1.x) {
                robot.reverseIntake();
                robot.toggleIntakeClaws();
                sleep(500);
                robot.stopIntake();
            } else {
                robot.stopIntake();
            }

            /* GAMEPAD 2 CONTROLS */

            if(gamepad2.left_bumper) {
                robot.setLiftPosition(Team8740_Base.LiftPosition.BOTTOM);
            } else if(gamepad2.left_trigger > 0.2) {
                robot.setLiftPosition(Team8740_Base.LiftPosition.MIDDLE);
            } else if(gamepad2.right_bumper) {
                robot.setLiftPosition(Team8740_Base.LiftPosition.TOP);
            }

            if(gamepad2.right_trigger > 0.2) {
                robot.toggleRelicWrist();
                sleep(100);
            }

            robot.setLiftPower(-gamepad2.right_stick_y);

            if(gamepad2.a) {
                robot.extendRelicArm();
            } else if(gamepad2.b) {
                robot.retractRelicArm();
            } else {
                robot.stopRelicArm();
            }

            if(gamepad2.x) {
                robot.toggleRelicClaw();
                sleep(75);
            }

            if(gamepad2.y) {
                robot.toggleProgramAssist();
                sleep(75);
            }

            // Pause for 10 mS each cycle = update 100 times a second.
            sleep(10);

            telemetry.addData("Lift encoder", robot.getLiftEncoder());
            telemetry.addData("Lift position", robot.getLiftPosition());
            telemetry.addData("Gyro position", robot.getGyroHeading());
            telemetry.addData("Distance", robot.getDistance());
            telemetry.addData("Program assist", robot.assistEnabled());
            telemetry.addLine("Encoders").addData("X", robot.getXPosition()).addData("Y", robot.getYPosition());
            telemetry.update();
        }
    }
}
