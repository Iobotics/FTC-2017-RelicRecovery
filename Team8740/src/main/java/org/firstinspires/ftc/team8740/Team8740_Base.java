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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

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

/**
 * Team 8740 base class
 *
 * @author Darren Kam
 */
public class Team8740_Base {
    /* Constants */
    private final static double LEFT_SERVO_HOME  = 0.3;
    private final static double RIGHT_SERVO_HOME = 0.6;
    private final static double LEFT_SERVO_CLOSED  = 0.33;
    private final static double RIGHT_SERVO_CLOSED = 0.63;
    private final static double LEFT_SERVO_OPEN  = 0.72;
    private final static double RIGHT_SERVO_OPEN = 1.25;

    private final static double JEWEL_SERVO_HOME = 0.45;
    private final static double JEWEL_SERVO_DOWN = 0.85;

    private final static double LOW_SPEED = 0.4;
    private final static double HIGH_SPEED = 0.7;

    private final static double INTAKE_POWER = 1.0;

    private final static double GLYPH_POWER = 1.0;

    private final static double LIFT_DOWN_POWER = -0.4;
    private final static double LIFT_UP_POWER = 0.6;

    private final static double TICKS_PER_REV   = 537.6;    // Ticks per revolution
    private final static double GEAR_REDUCTION  = 1.0;     // This is < 1.0 if geared UP
    private final static double WHEEL_DIAMETER  = 4.0;     // Inches
    private final static double TICKS_PER_INCH = (TICKS_PER_REV * GEAR_REDUCTION) * Math.sqrt(2) / (WHEEL_DIAMETER * 3.1415);

    private final static double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    private final static double P_TURN_COEFF       = 0.05;     // Larger is more responsive, but also less stable
    private final static double P_DRIVE_COEFF      = 0.05;    // Larger is more responsive, but also less stable

    // TODO - Find thresholds
    private final static double RED_THRESHOLD = 180;
    private final static double BLUE_THRESHOLD = 150;

    private final static String VUFORIA_LICENSE = "AY0QHQL/////AAAAGddY2lrlhEkenq0T04cRoVVmq/FAquH7DThEnayFrV+ojyjel8qTCn03vKe+FaZt0FwnE4tKdbimF0i47pzVuCQm2lRVdy5m1W03vvMN+8SA0RoXquxc1ddQLNyw297Ei3yWCJLV74UsEtfBwYKqr4ys3d2b2vPgaWnaZX6SNzD+x7AfKsaTSEIFqWfH8GOBoyw0kJ6qSCL384ylCcId6fVJbO8s9WccvuQYsCgCizdr0N/wOdEn76wY7fiNuR+5oReDCaIgfw5L35mD8EtQ0UHmNZGeDndtPDd6ZfNVlU3gyzch7nj5cmPBTleaoiCjyR9AputQHRH3qXnf3k76MvozmMGTE/j5o1HBA6BMSPwH";

    private enum Color {
        RED,
        BLUE,
        UNKNOWN
    }


    /* OpMode members */
    private DcMotor frontLeftDrive  = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive   = null;
    private DcMotor backRightDrive  = null;

    private DcMotor intakeLeft  = null;
    private DcMotor intakeRight = null;

    private DcMotor lift = null;

    private Servo leftServo  = null;
    private Servo rightServo = null;

    private Servo jewelServo = null;

    private CRServo pushServo = null;

    private DigitalChannel lowerLimit = null;
    private DigitalChannel upperLimit = null;

    private ColorSensor color_sensor = null;

    // The IMU sensor object
    private BNO055IMU imu = null;

    // State used for updating telemetry
    private Orientation angles   = null;
    private Acceleration gravity = null;

    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables relicTrackables = null;
    private VuforiaTrackable relicTemplate = null;

    private HardwareMap hwMap = null;

    private LinearOpMode opmode = null;

    private ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    /* Local OpMode members. */
    private boolean isLowSpeed = false;
    private boolean jewelStatus = false;
    private boolean clawStatus = false;
    private boolean teleop = false;

    private double speedMultiplier = HIGH_SPEED;


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        this.init(hwMap, null);
    }

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
        if(!teleop) initColorSensor();
        //initVuforia();
        if(!teleop) initGyro();
    }

    private void initDrive() {
        // Define and initialize motors
        frontLeftDrive  = hwMap.dcMotor.get("frontLeft");
        frontRightDrive = hwMap.dcMotor.get("frontRight");
        backLeftDrive   = hwMap.dcMotor.get("backLeft");
        backRightDrive  = hwMap.dcMotor.get("backRight");

        // Reverse left motors
        if(teleop) {
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        }

        // Set all motors to brake mode
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to run without encoders
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to zero power
        this.setPower(0, 0, 0, 0);
    }

    private void initIntake() {
        // Define the intake motors
        intakeLeft  = hwMap.dcMotor.get("intakeLeft");
        intakeRight = hwMap.dcMotor.get("intakeRight");

        // Reverse the right intake motor
        intakeRight.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initLift() {
        // Define the lift motor
        lift = hwMap.dcMotor.get("lift");

        // Define limit switches
        lowerLimit = hwMap.digitalChannel.get("lowerLimit");
        upperLimit = hwMap.digitalChannel.get("upperLimit");

        // Set limit switches to input mode
        lowerLimit.setMode(DigitalChannel.Mode.INPUT);
        upperLimit.setMode(DigitalChannel.Mode.INPUT);
    }

    private void initServos() {
        // Define and initialize ALL installed servos
        leftServo  = hwMap.servo.get("grabLeft");
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

    private void initColorSensor() {
        color_sensor = hwMap.get(ColorSensor.class, "color");
    }

    private void initVuforia() {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_LICENSE;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
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
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
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

    /**
     * Converts inches to ticks
     * @param inches
     * @return ticks
     */
    public double inchesToTicks(double inches) {
        double ticks = inches * TICKS_PER_INCH;
        return ticks;
    }

    public void setMode(DcMotor.RunMode mode) {
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

    public void lowerLift() {
        if(!lowerLimit.getState()) {
            lift.setPower(0);
        } else {
            lift.setPower(LIFT_DOWN_POWER);
        }
    }

    public double getLiftEncoder() {
        return lift.getCurrentPosition();
    }

    public void raiseLift() {
        if(!upperLimit.getState()) {
            lift.setPower(0);
        } else {
            lift.setPower(LIFT_UP_POWER);
        }
    }

    public void stopLift() {
        lift.setPower(0);
    }

    /**
     * Gets the state of the lower limit switch
     *
     * @return lowerLimitState
     */
    public boolean getLowerLimit() {
        boolean lowerLimitState = lowerLimit.getState();
        return lowerLimitState;
    }

    /**
     * Gets the state of the upper limit switch
     *
     * @return upperLimitState
     */
    public boolean getUpperLimit() {
        boolean upperLimitState = upperLimit.getState();
        return upperLimitState;
    }

    /**
     * Checks if claws are open
     *
     * @return clawStatus
     */
    public boolean getClawStatus() {
        return clawStatus;
    }

    public void toggleClaws() {
        leftServo.setPosition(clawStatus ? LEFT_SERVO_CLOSED : LEFT_SERVO_OPEN);
        rightServo.setPosition(clawStatus ? RIGHT_SERVO_CLOSED : RIGHT_SERVO_OPEN);
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

    public void toggleJewelArm() {
        jewelServo.setPosition(jewelStatus ? JEWEL_SERVO_DOWN : JEWEL_SERVO_HOME);
        jewelStatus = !jewelStatus;
    }

    /**
     * Gets color from the color sensor
     * @return color
     */
    public Color getColor() {
        Color color;
        if(color_sensor.red() > RED_THRESHOLD) {
            color = Color.RED;
        } else if(color_sensor.blue() > BLUE_THRESHOLD) {
            color = Color.BLUE;
        } else {
            color = Color.UNKNOWN;
        }

        return color;
    }

    public void activateVuforia() {
        relicTrackables.activate();
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
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
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
        }
        else {
            opmode.telemetry.addData("VuMark", "not visible");
        }

        opmode.telemetry.update();
    }

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
     * Checks if the gyro is calibrating
     *
     * @return isCalibrating
     */
    public boolean isGyroCalibrating() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        boolean isCalibrating = imu.isGyroCalibrated();

        return isCalibrating;
    }

    /**
     * Gets the heading of the gyro in degrees
     *
     * @return heading
     */
    public double getGyroHeading() {
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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
            moveCounts = distance * TICKS_PER_INCH;
            newLeftTarget = getLeftEncoder() + moveCounts;
            newRightTarget = getRightEncoder() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            setLeftEncoder(newLeftTarget);
            setRightEncoder(newRightTarget);

            setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

                leftSpeed  = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                setTank(leftSpeed, rightSpeed);

                // Display drive status for the driver.
                opmode.telemetry.addData("Err/St",  "%5.1f/%5.1f", error, steer);
                opmode.telemetry.addData("Target",  "%.2f:%.2f",      newLeftTarget,  newRightTarget);
                opmode.telemetry.addData("Actual",  "%.2f:%.2f",      getLeftEncoder(), getRightEncoder());
                opmode.telemetry.addData("Speed",   "%5.2f:%5.2f", leftSpeed, rightSpeed);
                opmode.telemetry.update();
            }

            // Stop all motion;
            setTank(0, 0);

            // Turn off RUN_TO_POSITION
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed  = -rightSpeed;
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
        while (robotError > 180)   robotError -= 360;
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
