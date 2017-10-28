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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Team 8740 base class
 * @author Darren Kam
 */
public class Team8740_Base
{
    /* Constants */
    private final static double LEFT_SERVO_HOME = 0.36;
    private final static double RIGHT_SERVO_HOME = 0.65;
    private final static double LEFT_SERVO_OPEN  = 0.72;
    private final static double RIGHT_SERVO_OPEN  = 1.25;

    private final static double JEWEL_SERVO_HOME  = 0.5;
    private final static double JEWEL_SERVO_DOWN  = 0.8;

    private final static double LOW_SPEED = 0.5;
    private final static double HIGH_SPEED = 0.7;

    private final static double INTAKE_POWER = 1.0;

    private final static double GLYPH_POWER = 1.0;

    private final static double LIFT_DOWN_POWER = -0.4;
    private final static double LIFT_UP_POWER = 0.6;


    /* OpMode members */
    private DcMotor frontLeftDrive   = null;
    private DcMotor frontRightDrive  = null;
    private DcMotor backLeftDrive    = null;
    private DcMotor backRightDrive   = null;

    private DcMotor intakeLeft = null;
    private DcMotor intakeRight = null;

    private DcMotor lift = null;

    private Servo leftServo = null;
    private Servo rightServo = null;

    private Servo jewelServo = null;

    private CRServo pushServo = null;

    private DigitalChannel lowerLimit = null;
    private DigitalChannel upperLimit = null;

    // The IMU sensor object
    private BNO055IMU imu = null;

    // State used for updating telemetry
    private Orientation angles = null;
    private Acceleration gravity = null;


    /* Local OpMode members. */
    private boolean isLowSpeed = false;
    private boolean clawStatus = false;

    private double speedMultiplier = HIGH_SPEED;

    private ElapsedTime time  = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private HardwareMap hwMap = null;


    /* Constructor */
    public Team8740_Base() { }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        initDrive();
        initIntake();
        initLift();
        initServos();
        //initGyro();
    }

    private void initDrive() {
        // Define and initialize motors
        frontLeftDrive  = hwMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hwMap.get(DcMotor.class, "frontRight");
        backLeftDrive   = hwMap.get(DcMotor.class, "backLeft");
        backRightDrive  = hwMap.get(DcMotor.class, "backRight");

        // Reverse left motors
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to brake mode
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to run without encoders
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to zero power
        this.setPower(0, 0, 0, 0);
    }

    private void initIntake() {
        // Define the intake motors
        intakeLeft = hwMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hwMap.get(DcMotor.class, "intakeRight");

        // Reverse the right intake motor
        intakeRight.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initLift() {
        // Define the lift motor
        lift = hwMap.get(DcMotor.class, "lift");

        // Define limit switches
        lowerLimit = hwMap.get(DigitalChannel.class, "lowerLimit");
        upperLimit = hwMap.get(DigitalChannel.class, "upperLimit");

        // Set limit switches to output mode
        lowerLimit.setMode(DigitalChannel.Mode.OUTPUT);
        upperLimit.setMode(DigitalChannel.Mode.OUTPUT);
    }

    private void initServos() {
        // Define and initialize ALL installed servos
        leftServo  = hwMap.get(Servo.class, "grabLeft");
        rightServo = hwMap.get(Servo.class, "grabRight");

        jewelServo = hwMap.get(Servo.class, "jewelArm");

        pushServo = hwMap.get(CRServo.class, "pushServo");

        // Reverse the right servo and jewel servo
        rightServo.setDirection(Servo.Direction.REVERSE);

        jewelServo.setDirection(Servo.Direction.REVERSE);

        // Home the servos
        leftServo.setPosition(LEFT_SERVO_HOME);
        rightServo.setPosition(RIGHT_SERVO_HOME);

        jewelServo.setPosition(JEWEL_SERVO_HOME);
    }

    private void initGyro() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        calibrateGyro();
    }

    /**
     * Set tank drive
     * @param left
     * @param right
     */
    public void setTank(double left, double right) {
        this.setPower(left, right, left, right);
    }

    /**
     * Set mecanum drive
     * @param x
     * @param y
     * @param rotation
     */
    public void setMecanum(double x, double y, double rotation) {
        // Setup a variable for each drive wheel to save power level for telemetry
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        frontLeftPower   = Range.clip(x + y - rotation, -1.0, 1.0);
        frontRightPower  = Range.clip(-x + y + rotation, -1.0, 1.0);
        backLeftPower    = Range.clip(-x + y - rotation, -1.0, 1.0);
        backRightPower   = Range.clip(x + y + rotation, -1.0, 1.0);

        // Send calculated power to wheels
        setPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public void setPower(double frontLeft, double frontRight, double backLeft, double backRight) {
        frontLeftDrive.setPower(frontLeft * speedMultiplier);
        frontRightDrive.setPower(frontRight * speedMultiplier);
        backLeftDrive.setPower(backLeft * speedMultiplier);
        backRightDrive.setPower(backRight * speedMultiplier);
    }

    /**
     * Toggles the speed of the bot
     */
    public void toggleSpeed() {
        speedMultiplier = isLowSpeed ? HIGH_SPEED : LOW_SPEED;
        isLowSpeed = !isLowSpeed;
    }

    public void runIntake() {
        intakeLeft.setPower(INTAKE_POWER);
        intakeRight.setPower(INTAKE_POWER);
    }

    public void reverseIntake() {
        intakeLeft.setPower(-INTAKE_POWER);
        intakeRight.setPower(-INTAKE_POWER);
    }


    public void stopIntake() {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }

    /**
     * Toggles the claws
     */
    public void toggleClaws() {
        leftServo.setPosition(clawStatus ? LEFT_SERVO_HOME : LEFT_SERVO_OPEN);
        rightServo.setPosition(clawStatus ? RIGHT_SERVO_HOME : RIGHT_SERVO_OPEN);
        clawStatus = !clawStatus;
    }

    public void pushGlyph() {
        pushServo.setPower(GLYPH_POWER);
    }

    public void retractGlyph() {
        pushServo.setPower(-GLYPH_POWER);
    }

    public void stopGlyph() {
        pushServo.setPower(0);
    }

    public void lowerLift() {
        //if(!lowerLimit.getState()) {
            lift.setPower(LIFT_DOWN_POWER);
        /*} else {
            lift.setPower(0);
        }*/
    }

    public void raiseLift() {
        //if(!upperLimit.getState()) {
            lift.setPower(LIFT_UP_POWER);
        /*} else {
            lift.setPower(0);
        }*/
    }

    public void stopLift() {
        lift.setPower(0);
    }

    /**
     * Gets the state of the lower limit switch
     * @return lowerLimitState
     */
    public boolean getLowerLimit() {
        boolean lowerLimitState = lowerLimit.getState();
        return lowerLimitState;
    }

    /**
     * Gets the state of the upper limit switch
     * @return upperLimitState
     */
    public boolean getUpperLimit() {
        boolean upperLimitState = upperLimit.getState();
        return upperLimitState;
    }

    public void lowerJewelArm() {
        jewelServo.setPosition(JEWEL_SERVO_DOWN);
    }

    public void raiseJewelArm() {
        jewelServo.setPosition(JEWEL_SERVO_HOME);
    }

    /**
     * Checks if claws are open
     * @return clawStatus
     */
    public boolean getClawStatus() {
        return clawStatus;
    }

    /**
     * Calibrates the gyro
     */
    public void calibrateGyro() {
        // Get the calibration data
        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();

        // Save the calibration data to a file. You can choose whatever file
        // name you wish here, but you'll want to indicate the same file name
        // when you initialize the IMU in an opmode in which it is used. If you
        // have more than one IMU on your robot, you'll of course want to use
        // different configuration file names for each.
        String filename = "BNO055IMUCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());
    }

    /**
     * Gets the calibration status of the gyro
     * @return
     */
    public String getGyroStatus() {
        String status = imu.getCalibrationStatus().toString();
        return status;
    }

    /**
     * Gets the heading of the gyro in degrees
     * @return heading
     */
    public double getGyroHeading() {
        double heading = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        return heading;
    }
}
