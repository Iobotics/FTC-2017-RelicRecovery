package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftc.vision.BeaconColorResult;
import ftc.vision.FrameGrabber;
import ftc.vision.ImageProcessorResult;

    @TeleOp(name = "TestVision", group = "Vision")
    @Disabled
    public class OpenCVTeleOp extends LinearOpMode {

        FrameGrabber frameGrabber = FtcRobotControllerActivity.frameGrabber;

        public void runOpMode() throws InterruptedException {

            waitForStart();
            while(opModeIsActive()) {
                if(gamepad1.x) {
                    frameGrabber.grabSingleFrame(); //Tell it to grab a frame
                    while (!frameGrabber.isResultReady()) { //Wait for the result
                        Thread.sleep(5); //sleep for 5 milliseconds
                    }

                    //Get the result
                    ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
                    BeaconColorResult result = (BeaconColorResult) imageProcessorResult.getResult();
                    BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
                    BeaconColorResult.BeaconColor rightColor = result.getRightColor();
                    telemetry.addData("LeftResult", leftColor); //Display it on telemetry
                    telemetry.addData("RightResult", rightColor); //Display it on telemetry
                    telemetry.update();
                    //wait before quitting (quitting clears telemetry)
                    Thread.sleep(1000);
                }
            }

        }

    }
