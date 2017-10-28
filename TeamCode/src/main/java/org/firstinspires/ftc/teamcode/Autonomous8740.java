package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Teacher on 10/16/2017.
 */

@Autonomous(name = "Autonomous8740", group = "Autonomous")
@Disabled
public class Autonomous8740 extends LinearOpMode {

    DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");



    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        runMotor();
    }

    public void runMotor(){
        frontLeft.setPower(1);
        backLeft.setPower(1);
        frontRight.setPower(1);
        backRight.setPower(1);
    }


}

