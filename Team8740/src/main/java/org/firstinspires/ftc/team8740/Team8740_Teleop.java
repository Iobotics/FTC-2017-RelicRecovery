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
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Team 8740: Teleop Mecanum", group="Team 8740")
//@Disabled
public class Team8740_Teleop extends LinearOpMode {

    Team8740_Base robot = new Team8740_Base();

    @Override
    public void runOpMode() {

        //Initialize the hardware variables.
        robot.init(hardwareMap, this);

        // Send telemetry message to signify robot waiting
        telemetry.addData("Say", "Waiting for start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Mecanum drive uses left stick to strafe along X and Y, and right stick X to turn.
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;

            double rotation = gamepad1.right_stick_x;

            robot.setMecanum(x, y, rotation);

            // Use gamepad A to open and close the claw
            if(gamepad1.a) {
                robot.toggleClaws();
                sleep(100);
            }

            // Use gamepad right bumper and trigger to run intake
            if(gamepad1.right_bumper) {
               robot.runIntake();
            } else if(gamepad1.right_trigger > 0.2) {
                robot.reverseIntake();
            }else {
                robot.stopIntake();
            }

            // Use gamepad Y and B to lower and raise the jewel arm
            if(gamepad1.y) {
                robot.lowerJewelArm();
            } else if(gamepad1.b) {
                robot.raiseJewelArm();
            }

            // Use gamepad left bumper and trigger to push the glyph
            if(gamepad1.left_bumper) {
                robot.pushGlyph();
            } else if(gamepad1.left_trigger > 0.2) {
                robot.retractGlyph();
            } else {
                robot.stopGlyph();
            }

            // Use gamepad dpad to control the lift
            if(gamepad1.dpad_down) {
                robot.lowerLift();
            } else if(gamepad1.dpad_up) {
                robot.raiseLift();
            } else {
                robot.stopLift();
            }

            // Use gamepad start to toggle speed control
            if(gamepad1.start) {
                robot.toggleSpeed();
            }

            // Send telemetry message to signify robot running;
            telemetry.addLine("limits").addData("lower", robot.getLowerLimit()).addData("upper", robot.getUpperLimit());
            //telemetry.addData("gyro status", robot.getGyroStatus());
            //telemetry.addData("heading", robot.getGyroHeading());
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}
