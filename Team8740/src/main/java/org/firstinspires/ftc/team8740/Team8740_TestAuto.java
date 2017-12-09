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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import ftc.vision.JewelColorResult;

@Autonomous(name = "Team 8740: Test Auto", group = "Team 8740")
//@Disabled
public class Team8740_TestAuto extends LinearOpMode {

    /* Declare OpMode members */
    Team8740_Base robot = new Team8740_Base();
    JewelColorResult.JewelColor color = null;
    //RelicRecoveryVuMark vuMark = null;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.4;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.4;     // Nominal half speed for better accuracy.

    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The initRobot() method of the hardware class does most of the work here
         */
        robot.initRobot(hardwareMap, this, JewelColorResult.JewelColor.BLUE);

        //robot.activateVuforia();

        // make sure the gyro is calibrated before continuing
        while (robot.isGyroCalibrating()) {
            if (robot.isStopRequested()) return;

            sleep(50);
            idle();
        }

        telemetry.addData("X", "Getting sensor data...");
        telemetry.update();

        color = robot.getColor();

        // Wait for the game to start (Display Gyro value), and reset gyro before we move
        while (!isStarted()) {
            if (robot.isStopRequested()) return;

            //vuMark = robot.getVuMark();

            telemetry.addData("O", "Robot Ready");
            telemetry.addLine("encoders").addData("X", robot.getXPosition()).addData("Y", robot.getYPosition());
            telemetry.addData(">", "Robot Heading = %.2f", robot.getGyroHeading());
            telemetry.addData("Color", color);
            //telemetry.addData("VuMark", vuMark);
            telemetry.update();
        }

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn

        robot.hitJewel(color);
        robot.driveOffBalance(false);
        robot.driveStraight(-16.0, 0.0);
        robot.gyroTurn(-90.0);
        robot.gyroHold(-90.0, 0.5);
        robot.driveStraight(6.0, -90.0);
        robot.releaseGlyph();
        robot.driveStraight(-4.0, -90.0);
    }
}