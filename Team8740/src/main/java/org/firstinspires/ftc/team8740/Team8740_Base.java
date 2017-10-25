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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Team8740_Base
{
    /* Public OpMode members. */
    private DcMotor frontLeftDrive   = null;
    private DcMotor frontRightDrive  = null;
    private DcMotor backLeftDrive    = null;
    private DcMotor backRightDrive   = null;

    public DcMotor intakeLeft = null;
    public DcMotor intakeRight = null;

    public DcMotor lift = null;

    public Servo leftServo = null;
    public Servo rightServo = null;

    public Servo jewelServo = null;

    public CRServo pushServo = null;

    public final static double LEFT_SERVO_HOME = 0.12;
    public final static double RIGHT_SERVO_HOME = 0.8;
    public final static double LEFT_SERVO_OPEN  = 0.63;
    public final static double RIGHT_SERVO_OPEN  = 0.9;

    public final static double JEWEL_SERVO_HOME  = 0.5;
    public final static double JEWEL_SERVO_DOWN  = 1.0;

    public final static double MAX_POWER = 0.8;

    public final static double INTAKE_POWER = 0.85;

    /* Local OpMode members. */
    private boolean clawsOpen = false;
    private ElapsedTime time  = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    /* Constructor */
    public Team8740_Base() { }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {

        // Define and Initialize Motors
        frontLeftDrive  = hwMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hwMap.get(DcMotor.class, "frontRight");
        backLeftDrive   = hwMap.get(DcMotor.class, "backLeft");
        backRightDrive  = hwMap.get(DcMotor.class, "backRight");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeLeft = hwMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hwMap.get(DcMotor.class, "intakeRight");

        intakeRight.setDirection(DcMotor.Direction.REVERSE);

        lift = hwMap.get(DcMotor.class, "lift");

        // Set all motors to zero power
        this.setPower(0, 0, 0, 0);

        // Set all motors to run without encoders
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos
        leftServo  = hwMap.get(Servo.class, "grabLeft");
        rightServo = hwMap.get(Servo.class, "grabRight");

        leftServo.setPosition(LEFT_SERVO_HOME);
        rightServo.setPosition(RIGHT_SERVO_HOME);

        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);

        jewelServo = hwMap.get(Servo.class, "jewelArm");

        //jewelServo.setPosition(JEWEL_SERVO_HOME);

        pushServo = hwMap.get(CRServo.class, "pushServo");
    }

    public void setTank(double left, double right) {
        this.setPower(left, right, left, right);
    }

    public void setPower(double frontLeft, double frontRight, double backLeft, double backRight) {
        frontLeftDrive.setPower(frontLeft * MAX_POWER);
        frontRightDrive.setPower(frontRight * MAX_POWER);
        backLeftDrive.setPower(backLeft * MAX_POWER);
        backRightDrive.setPower(backRight * MAX_POWER);
    }

    public void runIntake() {
        intakeLeft.setPower(INTAKE_POWER);
        intakeRight.setPower(INTAKE_POWER);
    }

    public void stopIntake() {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }

    public void toggleClaws() {
        leftServo.setPosition(clawsOpen ? LEFT_SERVO_HOME : LEFT_SERVO_OPEN);
        rightServo.setPosition(clawsOpen ? RIGHT_SERVO_HOME : RIGHT_SERVO_OPEN);
        clawsOpen = !clawsOpen;
    }

    public void lowerJewelArm() {
        jewelServo.setPosition(JEWEL_SERVO_DOWN);
    }

    public void resetJewelArm() {
        jewelServo.setPosition(JEWEL_SERVO_HOME);
    }

    public boolean areClawsOpen() {
        return clawsOpen;
    }
}
