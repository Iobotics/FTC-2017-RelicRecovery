package org.firstinspires.ftc.teamcode;
//Jack is awesome is a lie, this is the only true comment lol
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Jack Gonser on 9/29/2017.
 */

public class JackIsAwesome extends LinearOpMode {
    private DcMotor frontRightMotor = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor backLeftMotor = null;

    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right");

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Dab On It and Lets Go!", "Initialized");
        telemetry.update();
        //DAB on the haters!
        waitForStart();
        while (opModeIsActive()) {
            frontRightMotor.setPower(gamepad1.right_stick_y);
            frontLeftMotor.setPower(gamepad1.left_stick_y);
            backRightMotor.setPower(gamepad1.right_stick_y);
            backLeftMotor.setPower(gamepad1.left_stick_y);
            if (gamepad1.a == true) {
                backLeftMotor.setPower(-1.0);
                backRightMotor.setPower(1.0);
                frontLeftMotor.setPower(-1.0);
                frontRightMotor.setPower(1.0);
            }
        }
    }
}