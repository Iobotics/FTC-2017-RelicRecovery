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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import ftc.vision.FrameGrabber;
import ftc.vision.ImageProcessorResult;
import ftc.vision.JewelColorResult;

/**
 * Team 8740 base class
 *
 * @author Darren Kam
 */
public class Team8740_Base {
    /* Constants */
    public final static double LEFT_SERVO_HOME = 0.3;
    public final static double RIGHT_SERVO_HOME = 0.6;

    //public final static double LEFT_SERVO_CLOSED = 0.33;
    //public final static double RIGHT_SERVO_CLOSED = 0.63;

    public final static double LEFT_SERVO_OPEN = 0.72;
    public final static double RIGHT_SERVO_OPEN = 1.25;

    public final static double JEWEL_SERVO_HOME = 0.45;
    public final static double JEWEL_SERVO_DOWN = 0.85;

    // TODO - Find relic servo positions
    public final static double RELIC_WRIST_DOWN = 0.77;
    public final static double RELIC_WRIST_UP = 0.0;

    private final static double RELIC_CLAW_CLOSED = 0.0;
    private final static double RELIC_CLAW_OPEN = 1.0;

    private final static double LOW_SPEED = 0.5;
    private final static double HIGH_SPEED = 0.8;

    private final static double INTAKE_POWER = 1.0;

    private final static double GLYPH_POWER = 1.0;

    private final static double LIFT_POWER = 0.6;

    private final static double RELIC_ARM_SPEED = 0.5;

    private final static double LIFT_TICKS_PER_REV = 28;     // Ticks per revolution
    private final static double LIFT_GEAR_REDUCTION = 100.0; // This is < 1.0 if geared UP
    private final static double LIFT_GEAR_DIAMETER = 1.504;  // Inches
    private final static double LIFT_TICKS_PER_INCH = (LIFT_TICKS_PER_REV * LIFT_GEAR_REDUCTION) / (LIFT_GEAR_DIAMETER * 3.1415);

    private final static double LIFT_POS_MIDDLE = 2; // 6.5 Inches

    private final static double DRIVE_TICKS_PER_REV = 1024;  // Ticks per revolution
    private final static double DRIVE_GEAR_REDUCTION = 1.0; // This is < 1.0 if geared UP
    private final static double DRIVE_WHEEL_DIAMETER = 1.5; // Inches
    private final static double DRIVE_TICKS_PER_INCH = (DRIVE_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (DRIVE_WHEEL_DIAMETER * 3.1415);

    private final static double HEADING_THRESHOLD = 1; // As tight as we can make it with an integer gyro
    private final static double P_TURN_COEFF = 0.05;   // Larger is more responsive, but also less stable
    private final static double P_DRIVE_COEFF = 0.05;  // Larger is more responsive, but also less stable

    private final static String VUFORIA_LICENSE = "AY0QHQL/////AAAAGddY2lrlhEkenq0T04cRoVVmq/FAquH7DThEnayFrV+ojyjel8qTCn03vKe+FaZt0FwnE4tKdbimF0i47pzVuCQm2lRVdy5m1W03vvMN+8SA0RoXquxc1ddQLNyw297Ei3yWCJLV74UsEtfBwYKqr4ys3d2b2vPgaWnaZX6SNzD+x7AfKsaTSEIFqWfH8GOBoyw0kJ6qSCL384ylCcId6fVJbO8s9WccvuQYsCgCizdr0N/wOdEn76wY7fiNuR+5oReDCaIgfw5L35mD8EtQ0UHmNZGeDndtPDd6ZfNVlU3gyzch7nj5cmPBTleaoiCjyR9AputQHRH3qXnf3k76MvozmMGTE/j5o1HBA6BMSPwH";

    public enum LiftPosition {
        TOP,
        MIDDLE,
        BOTTOM
    }

    public enum Team {
        RED_TEAM,
        BLUE_TEAM
    }

    /* OpMode members */
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotor intakeLeft = null;
    private DcMotor intakeRight = null;

    private DcMotor lift = null;

    private DcMotor relicArm = null;

    private Servo leftServo = null;
    private Servo rightServo = null;

    private Servo jewelServo = null;

    private Servo relicWrist = null;
    private Servo relicClaw = null;

    private CRServo pushServo = null;

    private DigitalChannel lowerLimit = null;
    private DigitalChannel upperLimit = null;

    private DistanceSensor proxSensor = null;

    // The IMU sensor object
    private BNO055IMU imu = null;

    // State used for updating telemetry
    private Orientation angles = null;
    private Acceleration gravity = null;

    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables relicTrackables = null;
    private VuforiaTrackable relicTemplate = null;

    /* Local OpMode members. */
    private HardwareMap hwMap = null;

    private LinearOpMode opmode = null;

    public ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private LiftPosition liftPosition = LiftPosition.BOTTOM;

    private boolean isLowSpeed = false;
    private boolean jewelArmUp = false;
    private boolean relicClawOpen = false;
    private boolean intakeClawOpen = false;
    private boolean programAssist = false;
    private boolean teleop = false;

    private double speedMultiplier = HIGH_SPEED;

    private double xPosition = 0.0;
    private double yPosition = 0.0;


    /* Initialize standard Hardware interfaces */
    public void initRobot(HardwareMap hwMap, LinearOpMode opmode) {
        this.initRobot(hwMap, opmode, false);
    }

    /* Initialize standard Hardware interfaces */
    public void initRobot(HardwareMap hwMap, LinearOpMode opmode, boolean teleop) {
        this.hwMap = hwMap;
        this.opmode = opmode;
        this.teleop = teleop;

        initDrive();
        initIntake();
        initLift();
        initServos();
        initGyro();
        initRelic();
        initProx();
        if (!teleop) {
            initVuforia();
        }
    }

    private void initDrive() {
        // Define and initialize motors
        frontLeftDrive = hwMap.dcMotor.get("frontLeft");
        frontRightDrive = hwMap.dcMotor.get("frontRight");
        backLeftDrive = hwMap.dcMotor.get("backLeft");
        backRightDrive = hwMap.dcMotor.get("backRight");

        // Reverse left motors
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to brake mode
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to run without encoders
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to zero power
        setPower(0, 0, 0, 0);
    }

    private void initIntake() {
        // Define the intake motors
        intakeLeft = hwMap.dcMotor.get("intakeLeft");
        intakeRight = hwMap.dcMotor.get("intakeRight");

        // Reverse the right intake motor
        intakeRight.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initLift() {
        // Define the lift motor
        lift = hwMap.dcMotor.get("lift");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setPower(0);

        // Define limit switches
        lowerLimit = hwMap.digitalChannel.get("lowerLimit");
        upperLimit = hwMap.digitalChannel.get("upperLimit");

        // Set limit switches to input mode
        lowerLimit.setMode(DigitalChannel.Mode.INPUT);
        upperLimit.setMode(DigitalChannel.Mode.INPUT);
    }

    private void initServos() {
        // Define and initialize ALL installed servos
        leftServo = hwMap.servo.get("grabLeft");
        rightServo = hwMap.servo.get("grabRight");
        jewelServo = hwMap.servo.get("jewelArm");
        pushServo = hwMap.crservo.get("pushServo");

        // Reverse the right servo and jewel servo
        rightServo.setDirection(Servo.Direction.REVERSE);
        jewelServo.setDirection(Servo.Direction.REVERSE);
        pushServo.setDirection(CRServo.Direction.REVERSE);

        // Home the servos
        leftServo.setPosition(LEFT_SERVO_HOME);
        rightServo.setPosition(RIGHT_SERVO_HOME);
        jewelServo.setPosition(JEWEL_SERVO_HOME);
    }

    private void initRelic() {
        relicWrist = hwMap.servo.get("relicWrist");
        relicClaw = hwMap.servo.get("relicClaw");
        relicArm = hwMap.dcMotor.get("relicArm");

        relicClaw.setDirection(Servo.Direction.REVERSE);

        relicWrist.setPosition(RELIC_WRIST_DOWN);
        relicClaw.setPosition(0);

        relicArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initProx() {
        proxSensor = hwMap.get(DistanceSensor.class, "prox");
    }

    private void initVuforia() {
        //int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_LICENSE;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
    }

    private void initGyro() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public boolean isStopRequested() {
        boolean stopRequested = opmode.isStopRequested();

        if (stopRequested) {
            deactivateVuforia();
        }

        return stopRequested;
    }

    public void toggleProgramAssist() {
        //TODO - Add program-assisted methods
        programAssist = !programAssist;
    }

    public boolean assistEnabled() {
        return programAssist;
    }

    /**
     * Set tank drive
     *
     * @param left
     * @param right
     */
    public void setTank(double left, double right) {
        this.setPower(left, right, left, right);
    }

    /**
     * Set mecanum drive
     *
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

        frontLeftPower = Range.clip(x + y - rotation, -1.0, 1.0);
        frontRightPower = Range.clip(-x + y + rotation, -1.0, 1.0);
        backLeftPower = Range.clip(-x + y - rotation, -1.0, 1.0);
        backRightPower = Range.clip(x + y + rotation, -1.0, 1.0);

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

    public double getXPosition() {
        return frontRightDrive.getCurrentPosition();
    }

    public double getYPosition() {
        return -frontLeftDrive.getCurrentPosition();
    }

    public boolean xTargetReached(double position) {
        return false;
    }

    public boolean yTargetReached(double position) {
        return false;
    }

    public void setXPosition(double position) {
        frontLeftDrive.setTargetPosition(Math.round((float) position));
    }

    public void setYPosition(double position) {
        frontRightDrive.setTargetPosition(Math.round((float) position));
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        frontLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
        backLeftDrive.setMode(mode);
        backRightDrive.setMode(mode);
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

    public void setIntakePower(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }

    public double getLiftEncoder() {
        return lift.getCurrentPosition();
    }

    public void lowerLift() {
        if (getLowerLimit()) {
            lift.setPower(0);
        } else {
            lift.setPower(-LIFT_POWER);
        }
    }

    public void raiseLift() {
        if (getUpperLimit()) {
            lift.setPower(0);
        } else {
            lift.setPower(LIFT_POWER);
        }
    }

    public void stopLift() {
        lift.setPower(0);
    }

    public void setLiftPower(double power) {
        lift.setPower(power);
    }

    public void setLiftPosition(LiftPosition position) {
        liftPosition = position;
        switch (position) {
            case TOP:
                while (!getUpperLimit()) {
                    if (isStopRequested()) return;
                    lift.setPower(LIFT_POWER);
                }
                lift.setPower(0);
                break;
            case MIDDLE:
                int ticks = Math.round((float) (LIFT_TICKS_PER_INCH * LIFT_POS_MIDDLE));
                lift.setTargetPosition(ticks);

                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                lift.setPower(LIFT_POWER);

                while (lift.isBusy()) {
                    if (isStopRequested()) return;
                }

                lift.setPower(0);

                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            case BOTTOM:
                while (!getLowerLimit()) {
                    if (isStopRequested()) return;
                    lift.setPower(-LIFT_POWER);
                }
                lift.setPower(0);
                break;
        }
    }

    public LiftPosition getLiftPosition() {
        return liftPosition;
    }

    /**
     * Gets the state of the lower limit switch
     *
     * @return lowerLimitState
     */
    public boolean getLowerLimit() {
        boolean lowerLimitState = !lowerLimit.getState();
        return lowerLimitState;
    }

    /**
     * Gets the state of the upper limit switch
     *
     * @return upperLimitState
     */
    public boolean getUpperLimit() {
        boolean upperLimitState = !upperLimit.getState();
        return upperLimitState;
    }

    /**
     * Checks if intake claws are open
     *
     * @return intakeClawOpen
     */
    public boolean getIntakeClawStatus() {
        return intakeClawOpen;
    }

    public void toggleIntakeClaws() {
        leftServo.setPosition(intakeClawOpen ? LEFT_SERVO_HOME : LEFT_SERVO_OPEN);
        rightServo.setPosition(intakeClawOpen ? RIGHT_SERVO_HOME : RIGHT_SERVO_OPEN);
        intakeClawOpen = !intakeClawOpen;
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

    /**
     * Checks if the jewel arm is up
     *
     * @return jewelArmUp
     */
    public boolean getJewelArmStatus() {
        return jewelArmUp;
    }

    public void toggleJewelArm() {
        jewelServo.setPosition(jewelArmUp ? JEWEL_SERVO_DOWN : JEWEL_SERVO_HOME);
        jewelArmUp = !jewelArmUp;
    }

    public void extendRelicArm() {
        relicArm.setPower(RELIC_ARM_SPEED);
    }

    public void retractRelicArm() {
        relicArm.setPower(-RELIC_ARM_SPEED);
    }

    public void stopRelicArm() {
        relicArm.setPower(0);
    }

    public void setRelicClaw(double position) {
        relicClaw.setPosition(position);
    }

    public void toggleRelicClaw() {
        relicClaw.setPosition(relicClawOpen ? RELIC_CLAW_CLOSED : RELIC_CLAW_OPEN);
        relicClawOpen = !relicClawOpen;
    }

    public void setRelicWrist(double position) {
        relicWrist.setPosition(position);
    }

    /**
     * Gets the distance from the prox sensor in inches
     *
     * @return distance
     */
    public double getDistance() {
        double distance = proxSensor.getDistance(DistanceUnit.INCH);

        return distance;
    }

    /**
     * Gets color from the color sensor
     *
     * @return color
     */
    public JewelColorResult.JewelColor getColor() {
        FrameGrabber frameGrabber = FtcRobotControllerActivity.frameGrabber; //Get the frameGrabber

        opmode.telemetry.addData("Checkpoint", "4");
        opmode.telemetry.update();

        frameGrabber.grabSingleFrame(); //Tell it to grab a frame

        opmode.telemetry.addData("Checkpoint", "5");
        opmode.telemetry.addData("FrameGrabber", !frameGrabber.isResultReady());
        opmode.telemetry.update();

        while (!frameGrabber.isResultReady()) { //Wait for the result
            if (isStopRequested()) return JewelColorResult.JewelColor.UNKNOWN;
            opmode.sleep(5); //sleep for 5 milliseconds
        }

        opmode.telemetry.addData("Checkpoint", "6");
        opmode.telemetry.update();

        //Get the result
        ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
        JewelColorResult result = (JewelColorResult) imageProcessorResult.getResult();

        JewelColorResult.JewelColor leftColor = result.getLeftColor();
        JewelColorResult.JewelColor rightColor = result.getRightColor();

        JewelColorResult.JewelColor color;
        if (leftColor == JewelColorResult.JewelColor.BLUE && rightColor == JewelColorResult.JewelColor.RED) {
            color = JewelColorResult.JewelColor.RED;
        } else if (leftColor == JewelColorResult.JewelColor.RED && rightColor == JewelColorResult.JewelColor.BLUE) {
            color = JewelColorResult.JewelColor.BLUE;
        } else {
            color = JewelColorResult.JewelColor.UNKNOWN;
        }

        return color;
    }

    /**
     * FIXME - The negative angles will make the robot go in a straight line or curve
     * This method will knock the opposite jewel to the team color
     */
    public void jewelKnock() {
        toggleJewelArm();
        if (1 == 1) {
            if (1 == 1) {
                gyroTurn(.6, 10);
                gyroTurn(.6, 0);
            } else {
                gyroTurn(.6, -10);
                gyroTurn(.6, 0);
            }
        } else if (1 == 2) {
            if (getColor().equals(JewelColorResult.JewelColor.RED)) {
                gyroTurn(.6, 10);
                gyroTurn(.6, -10);
            } else {
                gyroTurn(.6, 10);
                gyroTurn(.6, -10);
            }
        }
        toggleJewelArm();
    }

    public void activateVuforia() {
        relicTrackables.activate();
    }

    public void deactivateVuforia() {
        if (relicTrackables != null) relicTrackables.deactivate();
    }

    public RelicRecoveryVuMark getVuMark() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        return vuMark;
    }

    /**
     * Checks if the gyro is calibrating
     *
     * @return isCalibrating
     */
    public boolean isGyroCalibrating() {
        boolean isCalibrating = !imu.isGyroCalibrated();

        return isCalibrating;
    }

    /**
     * Gets the heading of the gyro in degrees
     *
     * @return heading
     */
    public double getGyroHeading() {
        // Update gyro
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();

        double heading = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        return heading;
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void driveStraight(double speed, double distance, double angle) {
        double newTarget;
        double moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        double direction = 1;
        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            moveCounts = distance * DRIVE_TICKS_PER_INCH;
            newTarget = getYPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            frontLeftDrive.setTargetPosition(Math.round((float) newTarget));

            if (getYPosition() > frontLeftDrive.getTargetPosition()) {
                direction = -1;
            }

            // Start motion
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            setTank(speed * direction, speed * direction);

            // Keep looping while we are still active, and the front right motor is running.
            while (opmode.opModeIsActive() && !((direction < 0 && getYPosition() < frontLeftDrive.getTargetPosition()) || (direction > 0 && getYPosition() > frontLeftDrive.getTargetPosition()))) {
                // Adjust relative speed based on heading error
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // If driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed + steer;
                rightSpeed = speed - steer;

                // Normalize speeds if either one exceeds +/- 1.0
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                setTank(leftSpeed * direction, rightSpeed * direction);

                // Display drive status for the driver.
                opmode.telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                opmode.telemetry.addData("Target", "%.2f", newTarget);
                opmode.telemetry.addData("Actual", "%.2f:%.2f", getXPosition(), getYPosition());
                opmode.telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed * direction, rightSpeed * direction);
                opmode.telemetry.update();
            }

            // Stop all motion
            setTank(0, 0);
        }
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for motion.
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void driveStrafe(double speed, double distance, double angle) {
        int newXTarget;
        int newYTarget;
        double xMoveCounts;
        double max;
        double error;
        double steer;
        double xSpeed;
        double ySpeed;
        double rotation;
        double direction = 1;
        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            xMoveCounts = distance * Math.sqrt(2) * DRIVE_TICKS_PER_INCH;
            newXTarget = Math.round((float) (getXPosition() + xMoveCounts));
            newYTarget = 0;

            if (frontRightDrive.getCurrentPosition() > newXTarget) {
                direction = -1;
            }

            // Start motion
            xSpeed = Range.clip(Math.abs(speed), 0.0, 1.0);
            ySpeed = 0;

            setMecanum(xSpeed, ySpeed, 0);

            // Keep looping while we are still active
            while (opmode.opModeIsActive() && !((direction < 0 && frontRightDrive.getCurrentPosition() < newXTarget) || (direction > 0 && frontRightDrive.getCurrentPosition() > newXTarget))) {
                // Adjust relative speed based on heading error
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // If driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                rotation = steer;

                // Normalize speeds if either one exceeds +/- 1.0
                max = Math.max(Math.abs(xSpeed), Math.abs(ySpeed));
                if (max > 1.0) {
                    xSpeed /= max;
                    ySpeed /= max;
                }

                setMecanum(xSpeed, ySpeed, rotation);

                // Display drive status for the driver.
                opmode.telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                opmode.telemetry.addData("Target", "%d:%d", newXTarget, newYTarget);
                opmode.telemetry.addData("Actual", "%.2f:%.2f", getXPosition(), getYPosition());
                opmode.telemetry.addData("Speed", "%5.2f:%5.2f", xSpeed, ySpeed);
                opmode.telemetry.update();
            }

            // Stop all motion
            setMecanum(0, 0, 0);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opmode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            opmode.telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opmode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            opmode.telemetry.update();
        }

        // Stop all motion;
        setTank(0, 0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = rightSpeed; //Ethan changed this to positive because with negative it would go straight/arc
        }

        // Send desired speeds to motors
        setTank(leftSpeed, rightSpeed);

        // Display it for the driver
        opmode.telemetry.addData("Target", "%5.2f", angle);
        opmode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opmode.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * Positive error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getGyroHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  positive = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void resetTimer() {
        time.reset();
    }

    public double getElapsedTime() {
        return time.milliseconds();
    }
}
