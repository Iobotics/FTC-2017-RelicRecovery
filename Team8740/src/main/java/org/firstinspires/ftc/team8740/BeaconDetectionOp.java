package org.firstinspires.ftc.team8740;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftc.vision.BeaconColorResult;
import ftc.vision.FrameGrabber;
import ftc.vision.ImageProcessorResult;
import ftc.vision.JewelColorResult;

/**
 * Created by vandejd1 on 9/7/16.
 * FTC Team EV 7393
 */
@TeleOp(name="Team 8740: Test OpenCV", group="Team 8740")
public class BeaconDetectionOp extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    FrameGrabber frameGrabber = FtcRobotControllerActivity.frameGrabber; //Get the frameGrabber

    frameGrabber.grabSingleFrame(); //Tell it to grab a frame
    while (!frameGrabber.isResultReady()) { //Wait for the result
      Thread.sleep(5); //sleep for 5 milliseconds
    }

    //Get the result
    ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
    JewelColorResult result = (JewelColorResult) imageProcessorResult.getResult();

    JewelColorResult.JewelColor leftColor = result.getLeftColor();
    JewelColorResult.JewelColor rightColor = result.getRightColor();

    telemetry.addData("Result", result); //Display it on telemetry
    telemetry.update();
    //wait before quitting (quitting clears telemetry)
    Thread.sleep(1000);
  }
}
