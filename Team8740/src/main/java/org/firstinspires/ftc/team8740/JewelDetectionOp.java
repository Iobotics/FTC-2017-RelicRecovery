package org.firstinspires.ftc.team8740;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftc.vision.FrameGrabber;
import ftc.vision.ImageProcessorResult;
import ftc.vision.JewelColorResult;

@TeleOp(name = "Team 8740: Test OpenCV", group = "Team 8740")
//@Disabled
public class JewelDetectionOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FrameGrabber frameGrabber = FtcRobotControllerActivity.frameGrabber; //Get the frameGrabber

        frameGrabber.grabSingleFrame(); //Tell it to grab a frame
        while (!frameGrabber.isResultReady()) { //Wait for the result
            if (isStopRequested()) return;

            sleep(5); //sleep for 5 milliseconds
        }

        //Get the result
        ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
        JewelColorResult result = (JewelColorResult) imageProcessorResult.getResult();

        telemetry.addData("Result", result); //Display it on telemetry
        telemetry.update();
        //wait before quitting (quitting clears telemetry)
        sleep(1000);
    }
}
