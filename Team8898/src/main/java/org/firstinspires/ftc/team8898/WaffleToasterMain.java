package org.firstinspires.ftc.team8898;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Jack Gonser :) on 11/27/2017.
 */

public class WaffleToasterMain {
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    // creates arm variable
    public DcMotor arm = null;
    // sets up variable for the arm servos
    public Servo leftServo = null;
    public Servo rightServo = null;
    // sets up variable for the jewel servo
    public Servo jewelServo = null;
    NormalizedColorSensor colorSensor;

    HardwareMap hwmap = null;

    private boolean teleOp;
    // Thanks Joel
    private final int TICKS_PER_REV = 1120;
    private final int WHEEL_DIAMETER = 4;
    private final double COUNTS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);
    private final static double P_TURN_COEFF = 0.07;     // Larger is more responsive, but also less stable
    private final static double P_DRIVE_COEFF = 0.01;
    private final int DRIVE_THRESHOLD = (int) (0.1 * COUNTS_PER_INCH);
    private LinearOpMode opMode = null;
    private BNO055IMU imu = null;
    private Orientation angles = null;
    private Acceleration gravity = null;
    private final static double HEADING_THRESHOLD = 1;

    public void WaffleToasterMain(boolean teleOpCheck, LinearOpMode opMode) {
        if (teleOpCheck) {
            teleOp = true;
        } else {
            teleOp = false;
        }
        this.opMode = opMode;
    }

    void init(HardwareMap awhmap, boolean autoBlue) {
        hwmap = awhmap;

        leftFront = hwmap.get(DcMotor.class, "leftFront");
        leftBack = hwmap.get(DcMotor.class, "leftBack");
        rightFront = hwmap.get(DcMotor.class, "rightFront");
        rightBack = hwmap.get(DcMotor.class, "rightBack");

        arm = hwmap.get(DcMotor.class, "arm");

        leftServo = hwmap.get(Servo.class, "leftServo");
        rightServo = hwmap.get(Servo.class, "rightServo");

        jewelServo = hwmap.get(Servo.class, "jewelServo");

        colorSensor = hwmap.get(NormalizedColorSensor.class, "colorSensor");

        if (autoBlue && !teleOp) {
            leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
            leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        } else if (teleOp) {
            //for butterfly drive
            leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
            leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        }
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);

        jewelServo.setDirection(Servo.Direction.FORWARD);
    }

    /**
     * Sleep from LinearOpMode
     *
     * @param milliseconds
     */
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Sets the two arm servos to a set position
     *
     * @param position
     */
    public void allServo(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }

    /**
     * Sets power to the four motors by left or right values
     *
     * @param leftPower
     * @param rightPower
     */
    public void allDrive(double leftPower, double rightPower) {
        leftBack.setPower(leftPower);
        leftFront.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);

    }

    /**
     * Turns the robot depending on the direction you set it to turn and the power you set it to go
     *
     * @param direction
     * @param speed
     */
    public void turnDrive(String direction, double speed) {
        direction = direction.toLowerCase();
        if (direction == "left") {
            leftFront.setPower(-speed);
            leftBack.setPower(-speed);
            rightFront.setPower(speed);
            rightBack.setPower(speed);
        } else if (direction == "right") {
            leftFront.setPower(speed);
            leftBack.setPower(speed);
            rightFront.setPower(-speed);
            rightBack.setPower(-speed);
        } else {
        }
    }

    /**
     * Resets robot to what you put in
     * drive stops the robot
     * arm lowers and opens the servos
     * jewel brings up the arm and turns the light off
     * all resets both the jewel and the drive motors
     *
     * @param resetParts
     */
    public void resetRobot(String resetParts) {
        resetParts = resetParts.toLowerCase();
        if (resetParts == "drive") {
            allDrive(0, 0);
        } else if (resetParts == "jewel") {
            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight) colorSensor).enableLight(false);
            }
            jewelServo.setPosition(1);
        } else if (resetParts == "all") {
            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight) colorSensor).enableLight(false);
            }
            jewelServo.setPosition(1);
            allDrive(0, 0);
        } else if (resetParts == "arm") {
            allServo(0);
            arm.setPower(0.4);
            sleep(200);
            arm.setPower(0);
        }
    }

    /**
     * Sets the speed for strafing and normal drive with a butterfly drive train
     *
     * @param speed
     * @param forward
     */
    public void twoMotorDrive(double speed, boolean forward) {
        if (forward) {
            leftFront.setPower(speed);
            rightBack.setPower(-speed);
        } else {
            leftBack.setPower(-speed);
            rightFront.setPower(speed);
        }
    }

    //Thanks Joel
    public void encoderDrive(LinearOpMode opmode, double inches, double maxSpeed) {
        double speed;
        int error;
        //sets the target encoder value
        int target = rightFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

        // While the absolute value of the error is greater than the error threshold
        while (opmode.opModeIsActive() && Math.abs(rightFront.getCurrentPosition() - target) >= DRIVE_THRESHOLD) {
            error = target - rightFront.getCurrentPosition();
            speed = Range.clip(error * P_DRIVE_COEFF, -maxSpeed, maxSpeed);

            allDrive(speed, speed);
        }
        resetRobot("drive");
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
        imu = hwmap.get(BNO055IMU.class, "imu");
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

        }
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
        allDrive(leftSpeed, rightSpeed);


        return onTarget;
    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getGyroHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }public double getGyroHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
        double heading = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        return heading;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}
