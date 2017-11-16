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

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

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
    private final static double LEFT_SERVO_HOME = 0.3;
    private final static double RIGHT_SERVO_HOME = 0.6;

    private final static double LEFT_SERVO_CLOSED = 0.33;
    private final static double RIGHT_SERVO_CLOSED = 0.63;

    private final static double LEFT_SERVO_OPEN = 0.72;
    private final static double RIGHT_SERVO_OPEN = 1.25;

    private final static double JEWEL_SERVO_HOME = 0.45;
    private final static double JEWEL_SERVO_DOWN = 0.85;

    // TODO - Find relic servo positions
    private final static double RELIC_SERVO_CLOSED = 0.0;
    private final static double RELIC_SERVO_OPEN = 1.0;

    private final static double LOW_SPEED = 0.4;
    private final static double HIGH_SPEED = 0.7;

    private final static double INTAKE_POWER = 1.0;

    private final static double GLYPH_POWER = 1.0;

    private final static double LIFT_POWER = 0.6;

    private final static double RELIC_ARM_SPEED = 0.5;

    private final static double LIFT_TICKS_PER_REV = 560;    // Ticks per revolution
    private final static double LIFT_GEAR_REDUCTION = 100.0; // This is < 1.0 if geared UP
    private final static double LIFT_GEAR_DIAMETER = 1.504;  // Inches
    private final static double LIFT_TICKS_PER_INCH = (LIFT_TICKS_PER_REV * LIFT_GEAR_REDUCTION) / (LIFT_GEAR_DIAMETER * 3.1415);

    private final static double LIFT_POS_MIDDLE = 6.5; /// Inches

    private final static double DRIVE_TICKS_PER_REV = 512;  // Ticks per revolution
    private final static double DRIVE_GEAR_REDUCTION = 1.0; // This is < 1.0 if geared UP
    private final static double DRIVE_WHEEL_DIAMETER = 4.0; // Inches
    private final static double DRIVE_TICKS_PER_INCH = (DRIVE_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) * Math.sqrt(2) / (DRIVE_WHEEL_DIAMETER * 3.1415);

    private final static double HEADING_THRESHOLD = 1; // As tight as we can make it with an integer gyro
    private final static double P_TURN_COEFF = 0.05;   // Larger is more responsive, but also less stable
    private final static double P_DRIVE_COEFF = 0.05;  // Larger is more responsive, but also less stable

    // TODO - Find thresholds
    private final static double RED_MIN_THRESHOLD = 140;
    private final static double RED_MAX_THRESHOLD = 180;

    private final static double BLUE_MIN_THRESHOLD = 150;
    private final static double BLUE_MAX_THRESHOLD = 150;

    private final static double STAGE_ONE_POSITION = 0;
    private final static double STAGE_TWO_POSITION = 0;
    private final static double STAGE_THREE_POSITION = 0;
    private final static double ONE_INCH_INTERVAL = 0;

    private final static String VUFORIA_LICENSE = "AY0QHQL/////AAAAGddY2lrlhEkenq0T04cRoVVmq/FAquH7DThEnayFrV+ojyjel8qTCn03vKe+FaZt0FwnE4tKdbimF0i47pzVuCQm2lRVdy5m1W03vvMN+8SA0RoXquxc1ddQLNyw297Ei3yWCJLV74UsEtfBwYKqr4ys3d2b2vPgaWnaZX6SNzD+x7AfKsaTSEIFqWfH8GOBoyw0kJ6qSCL384ylCcId6fVJbO8s9WccvuQYsCgCizdr0N/wOdEn76wY7fiNuR+5oReDCaIgfw5L35mD8EtQ0UHmNZGeDndtPDd6ZfNVlU3gyzch7nj5cmPBTleaoiCjyR9AputQHRH3qXnf3k76MvozmMGTE/j5o1HBA6BMSPwH";

    public enum Color {
        RED,
        BLUE,
        UNKNOWN
    }

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

    private Servo relicServo = null;

    private CRServo pushServo = null;

    private DigitalChannel lowerLimit = null;
    private DigitalChannel upperLimit = null;

    private ColorSensor color_sensor = null;

    // The IMU sensor object
    private BNO055IMU imu = null;

    // State used for updating telemetry
    private Orientation angles = null;
    private Acceleration gravity = null;

    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables relicTrackables = null;
    private VuforiaTrackable relicTemplate = null;

    private FrameGrabber frameGrabber = null;

    /* Local OpMode members. */
    private HardwareMap hwMap = null;

    private LinearOpMode opmode = null;

    private ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private LiftPosition liftPosition = LiftPosition.BOTTOM;

    private boolean isLowSpeed = false;
    private boolean jewelArmUp = false;
    private boolean relicClawOpen  = false;
    private boolean intakeClawOpen = false;
    private boolean teleop = false;

    private double speedMultiplier = HIGH_SPEED;


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap, LinearOpMode opmode) {
        this.init(hwMap, opmode, false);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap, LinearOpMode opmode, boolean teleop) {
        this.hwMap = hwMap;
        this.opmode = opmode;
        this.teleop = teleop;

        initDrive();
        initIntake();
        initLift();
        initServos();
        if (!teleop) {
            initRelic();
            initColorSensor();
            initVuforia();
            initGyro();
        }
    }

    private void initDrive() {
        // Define and initialize motors
        frontLeftDrive  = hwMap.dcMotor.get("frontLeft");
        frontRightDrive = hwMap.dcMotor.get("frontRight");
        backLeftDrive   = hwMap.dcMotor.get("backLeft");
        backRightDrive  = hwMap.dcMotor.get("backRight");

        // Reverse left motors
        if (teleop) {
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        }

        // Set all motors to brake mode
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to run without encoders
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
        opmode.idle();

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

        // Home the servos
        leftServo.setPosition(LEFT_SERVO_HOME);
        rightServo.setPosition(RIGHT_SERVO_HOME);

        jewelServo.setPosition(JEWEL_SERVO_HOME);
    }

    private void initRelic() {
        relicArm = hwMap.dcMotor.get("relicArm");

        relicArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initColorSensor() {
        frameGrabber = FtcRobotControllerActivity.frameGrabber;

        color_sensor = hwMap.get(ColorSensor.class, "color");
    }

    private void initVuforia() {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

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

        frontLeftPower  = Range.clip(x + y - rotation, -1.0, 1.0);
        frontRightPower = Range.clip(-x + y + rotation, -1.0, 1.0);
        backLeftPower   = Range.clip(-x + y - rotation, -1.0, 1.0);
        backRightPower  = Range.clip(x + y + rotation, -1.0, 1.0);

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

    public double getLeftEncoder() {
        return frontLeftDrive.getCurrentPosition();
    }

    public double getRightEncoder() {
        return frontRightDrive.getCurrentPosition();
    }

    public void setLeftEncoder(double position) {
        frontLeftDrive.setTargetPosition(Math.round((float) position));
    }

    public void setRightEncoder(double position) {
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
        if (!lowerLimit.getState()) {
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
                    lift.setPower(LIFT_POWER);
                }
                lift.setPower(0);
                break;
            case MIDDLE:
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                opmode.idle();
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                int ticks = Math.round((float) (LIFT_TICKS_PER_INCH * LIFT_POS_MIDDLE));
                lift.setTargetPosition(ticks);
                lift.setPower(LIFT_POWER);

                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            case BOTTOM:
                while (!getLowerLimit()) {
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
        leftServo.setPosition(intakeClawOpen ? LEFT_SERVO_CLOSED : LEFT_SERVO_OPEN);
        rightServo.setPosition(intakeClawOpen ? RIGHT_SERVO_CLOSED : RIGHT_SERVO_OPEN);
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

    public void stopRelicArm() {
        relicArm.setPower(0);
    }

    public void setRelicServo(double position) {
        relicServo.setPosition(position);
    }

    public void toggleRelicServo() {
        relicServo.setPosition(relicClawOpen ? RELIC_SERVO_CLOSED : RELIC_SERVO_OPEN);
        relicClawOpen = !relicClawOpen;
    }

    /**
     * Gets color from the color sensor
     *
     * @return color
     */
    public Color getColor() {
        frameGrabber.grabSingleFrame(); //Tell it to grab a frame
        while (!frameGrabber.isResultReady()) { //Wait for the result
            opmode.sleep(5); //sleep for 5 milliseconds
        }
        //Get the result
        ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
        JewelColorResult result = (JewelColorResult) imageProcessorResult.getResult();
        JewelColorResult.JewelColor leftColor = result.getLeftColor();
        JewelColorResult.JewelColor rightColor = result.getRightColor();

        Color color;
        if (leftColor == JewelColorResult.JewelColor.BLUE && rightColor == JewelColorResult.JewelColor.RED) {
            color = Color.RED;
        } else if (leftColor == JewelColorResult.JewelColor.RED && rightColor == JewelColorResult.JewelColor.BLUE) {
            color = Color.BLUE;
        } else {
            color = Color.UNKNOWN;
        }

        return color;
    }

    /** TODO - The negative angles will make the robot go in a straight line or curve
     * This method will knock the opposite jewel to the team color
     * @return false if it has not knocked it off the jewel to make
     */
    public boolean jewelKnock() {
        toggleJewelArm();
        if (1 == 1) {
            if (1==1) {
                opmode.telemetry.addData("test", "test");
                opmode.telemetry.update();
                gyroTurn(.6, 10);
                gyroTurn(.6, -10);
                return true;
            }
            else {
                gyroTurn(.6, 10);
                gyroTurn(.6, -10);
                return true;
            }
        }
        else if(1 == 2){
            if (getColor().equals(Color.RED)) {
                gyroTurn(.6, 10);
                gyroTurn(.6, -10);
                toggleJewelArm();
                return true;
            } else {
                gyroTurn(.6, 10);
                gyroTurn(.6, -10);
                toggleJewelArm();
                return true;
            }
        }
        toggleJewelArm();
        return false;
    }

    public void activateVuforia() {
        relicTrackables.activate();
    }

    public void deactivateVuforia() {
        relicTrackables.deactivate();
    }

    public void trackVuMarks() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            opmode.telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
            String formattedPose = (pose != null) ? pose.formatAsTransform() : "null";
            opmode.telemetry.addData("Pose", formattedPose);

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;
            }
        } else {
            opmode.telemetry.addData("VuMark", "not visible");
        }

        opmode.telemetry.update();
    }

    /**
     * Checks if the gyro is calibrating
     *
     * @return isCalibrating
     */
    public boolean isGyroCalibrating() {
        boolean isCalibrating = imu.isGyroCalibrated();

        return isCalibrating;
    }

    /**
     * Gets the heading of the gyro in degrees
     *
     * @return heading
     */
    public double getGyroHeading() {
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
    public void gyroDrive(double speed, double distance, double angle) {
        double newLeftTarget;
        double newRightTarget;
        double moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            moveCounts = distance * DRIVE_TICKS_PER_INCH;
            newLeftTarget = getLeftEncoder() + moveCounts;
            newRightTarget = getRightEncoder() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            setLeftEncoder(newLeftTarget);
            setRightEncoder(newRightTarget);

            setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            setTank(speed, speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opmode.opModeIsActive() && (frontLeftDrive.isBusy() && frontRightDrive.isBusy())) {
                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                setTank(leftSpeed, rightSpeed);

                // Display drive status for the driver.
                opmode.telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                opmode.telemetry.addData("Target", "%.2f:%.2f", newLeftTarget, newRightTarget);
                opmode.telemetry.addData("Actual", "%.2f:%.2f", getLeftEncoder(), getRightEncoder());
                opmode.telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                opmode.telemetry.update();
            }

            // Stop all motion;
            setTank(0, 0);

            // Turn off RUN_TO_POSITION
            setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            leftSpeed = rightSpeed; //Ethan- changed this to positive because with negative it would go straight/arc
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
}
