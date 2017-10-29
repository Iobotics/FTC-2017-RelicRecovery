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

package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Setting motors right here under
// USE setPosition to program Servos
@TeleOp(name = "TeleOp: Gold Digger Tank Drive", group = "Linear Opmode")
//@Disabled
public class GoldDiggerOpMode_Linear extends LinearOpMode {
    GoldDiggerBot robot = new GoldDiggerBot();

    // TANK DRIVE NOT ARCADE
    // Have an else for every if especially for
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            // Setting the motors to different joysticks of controller

                robot.drive(-gamepad1.left_stick_y* 0.5, -gamepad1.right_stick_y * 0.5);

            // If the letter "A" on gamepad is HELD, robot will spin
            if (gamepad1.a == true) {
                robot.glyphPull(-1, -1);
            } else {
                robot.glyphPull(0, 0);
            }


            if (gamepad1.b == true) {
                robot.glyphPull(1, 1);
            } else {
                robot.glyphPull(0, 0);
            }

            if (gamepad1.x){
                robot.elevatorLift(robot.bottomLift.getPosition() + 0.01);
                if(robot.bottomLift.getPosition() >= robot.MAX_POS){
                    robot.elevatorLift(robot.MAX_POS);
                }
                Thread.sleep(50);
            }
            if (gamepad1.y){
                robot.elevatorLift(robot.bottomLift.getPosition() - 0.01);
                if(robot.bottomLift.getPosition() <= robot.MIN_POS){
                    robot.elevatorLift(robot.MIN_POS);
                }
                Thread.sleep(50);
            }
            telemetry.addData("position", robot.bottomLift.getPosition());
            telemetry.update();

        }
    }
}
