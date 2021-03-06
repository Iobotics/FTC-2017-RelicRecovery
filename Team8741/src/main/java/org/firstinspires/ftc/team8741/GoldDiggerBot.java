package org.firstinspires.ftc.team8741;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

import ftc.vision.FrameGrabber;
import ftc.vision.JewelColorResult;

import static java.lang.Thread.sleep;


/**
 * Created by Joshua Taufahema on 10/13/2017.
 */
//Servo = An object designed to give the driver much more control, but less speed than motors
//Motor = An object designed to provide the driver a lot of speed, but less accuracy and precision than Servo
public class GoldDiggerBot {
    //initialises all motors and servos
    private DcMotor leftBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftGlyphPull = null;
    private DcMotor rightGlyphPull = null;
    public DcMotor conveyor = null;
    public Servo jewelServo = null;
    private ModernRoboticsI2cColorSensor colorSensor;

    private final int TICKS_PER_REV = 1120;
    private final int WHEEL_DIAMETER = 4;
    private final double COUNTS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);
    private final static double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    private final static double P_TURN_COEFF = 0.11;     // Larger is more responsive, but also less stable
    private final static double P_DRIVE_COEFF = 0.01;    // Larger is more responsive, but also less stable
    public final double JEWEL_ARM_DOWN = 0.76;
    public final double JEWEL_ARM_UP = 0.24;
    private final int DRIVE_THRESHOLD = (int) (0.1 * COUNTS_PER_INCH);


    private HardwareMap hwMap = null;
    private LinearOpMode opMode = null;
    private BNO055IMU imu = null;
    private Orientation angles = null;
    private Acceleration gravity = null;


    public GoldDiggerBot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    void init(HardwareMap ahwMap, boolean isAuto) {

        hwMap = ahwMap;

        //getting motors
        leftBackDrive = hwMap.get(DcMotor.class, "backLeft");
        leftFrontDrive = hwMap.get(DcMotor.class, "frontLeft");
        rightBackDrive = hwMap.get(DcMotor.class, "backRight");
        rightFrontDrive = hwMap.get(DcMotor.class, "frontRight");
        rightGlyphPull = hwMap.get(DcMotor.class, "rightIntake");
        leftGlyphPull = hwMap.get(DcMotor.class, "leftIntake");
        conveyor = hwMap.get(DcMotor.class, "conveyor");
        jewelServo = hwMap.get(Servo.class, "jewelServo");


        //setting direction of motors and ther behaviour

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightGlyphPull.setDirection(DcMotor.Direction.FORWARD);
        leftGlyphPull.setDirection(DcMotor.Direction.REVERSE);
        conveyor.setDirection(DcMotor.Direction.REVERSE);
        jewelServo.setDirection(Servo.Direction.FORWARD);

        /*sets Behaviour of the motors depending on the type of Opmode
        if it is auto then brake
        if it is TeleOp then use default
        TODO: Determine which is prefered, default or float
        */
        if (isAuto) {
            setZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
            initGyro();
            colorSensor =hwMap.get(ModernRoboticsI2cColorSensor.class, "colorSensor");
        } else {
            //use default zero power behaviour on teleop
        }
        opMode.waitForStart();
        jewelServo.setPosition(JEWEL_ARM_UP);
    }
    //set run mode of the motors

    public void setMode(DcMotor.RunMode mode) {
        leftBackDrive.setMode(mode);
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    //returns the encoder value on the left side
    public double getLeftEncoder() {
        return leftFrontDrive.getCurrentPosition();
    }

    //returns the encoder value on the right
    public double getRightEncoder() {
        return rightFrontDrive.getCurrentPosition();
    }


    public void setLeftEncoder(double position) {
        leftFrontDrive.setTargetPosition(Math.round((float) position));
    }

    public void setRightEncoder(double position) {
        rightFrontDrive.setTargetPosition(Math.round((float) position));
    }

    //set the behaviour for all drive motors
    public void setZeroPower(DcMotor.ZeroPowerBehavior behavior) {
        leftBackDrive.setZeroPowerBehavior(behavior);
        leftFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
    }

    //gives specified power to each side
    public void drive(double leftPower, double rightPower) {
        leftPower = Range.clip(leftPower, -1.0, 1.0);
        rightPower = Range.clip(rightPower, -1.0, 1.0);

        leftBackDrive.setPower(leftPower);
        leftFrontDrive.setPower(leftPower);
        rightBackDrive.setPower(rightPower);
        rightFrontDrive.setPower(rightPower);
    }

    //stops all motors
    public void stopDrive() {
        drive(0, 0);
    }

    public void glyphPull(double glyphPower) {
        leftGlyphPull.setPower(glyphPower);
        rightGlyphPull.setPower(glyphPower);
        //sets power to the intake
    }

    /**
     * Method for driving straight
     *
     * @param inches Inches
     * @param maxSpeed  Should be greater than 0. Sets the maximum possible speed value.
     */
    public void encoderDrive(LinearOpMode opmode, double inches, double maxSpeed) {
        double speed;
        int error;
        //sets the target encoder value
        int target = rightFrontDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

        // While the absolute value of the error is greater than the error threshold
        while (opmode.opModeIsActive() && Math.abs(rightFrontDrive.getCurrentPosition() - target) >= DRIVE_THRESHOLD) {
            error = target - rightFrontDrive.getCurrentPosition();
            speed = Range.clip(error * P_DRIVE_COEFF, -maxSpeed , maxSpeed);

            drive(speed, speed);
            opMode.telemetry.addData("speed: ", speed);
            opMode.telemetry.update();
        }
        stopDrive();
    }

    void initGyro() {
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

    public boolean isGyroCalibrating() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
        boolean isCalibrating = imu.isGyroCalibrated();

        return isCalibrating;
    }

    public double getGyroHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
        double heading = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        return heading;
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getGyroHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
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
        while (!onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            opMode.telemetry.update();
        }
    }
    public void displayColors(){
        opMode.telemetry.addData("Red: ", colorSensor.red());
        opMode.telemetry.addData("Blue: ", colorSensor.blue());
        opMode.telemetry.addData("Green: ", colorSensor.green());
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
        while ((holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            opMode.telemetry.update();
        }

        // Stop all motion;
        drive(0, 0);
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
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors
        drive(leftSpeed, rightSpeed);

        // Display it for the driver
        opMode.telemetry.addData("Target", "%5.2f", angle);
        opMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opMode.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public int knockJewel(JewelColorResult.JewelColor color) throws InterruptedException {
        jewelServo.setPosition(JEWEL_ARM_DOWN);
        sleep(1000);
        int returnVal = detectJewel(color);
        jewelServo.setPosition(JEWEL_ARM_UP);
        return  returnVal;
    }
    /**
     * Method checks the color of jewel infront of the robot
     * and then knocks the jewel over based on the color detected
     * and the side the robot is on.
     * @param color tells which side the robot is on.
     */
    private int detectJewel(JewelColorResult.JewelColor color){
        if(colorSensor.blue() > colorSensor.red() && color == JewelColorResult.JewelColor.RED || colorSensor.red() > colorSensor.blue() && color == JewelColorResult.JewelColor.BLUE){
            encoderDrive(opMode, 3, 0.4);

            return -3;
        }
        else {
            encoderDrive(opMode, -3, 0.4);
            return 3;
        }
    }
}

