package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Teacher on 10/4/2017.
 */
@TeleOp(name="Teleop8740", group="8740")
//@Disabled
public class Teleop8740 extends LinearOpMode {
    DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
    DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");



    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;




    @Override
    public void runOpMode() {

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        while (opModeIsActive()) {

            double rotation = gamepad1.right_stick_x;

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;

            frontLeftPower = Range.clip(x + y - rotation, -1.0, 1.0);
            frontRightPower = Range.clip(-x + y + rotation, -1.0, 1.0);
            backLeftPower = Range.clip(-x + y - rotation, -1.0, 1.0);
            backRightPower = Range.clip(x + y + rotation, -1.0, 1.0);

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(frontLeftPower);
            frontRight.setPower(frontLeftPower);
            backRight.setPower(frontLeftPower);




        }
    }

}