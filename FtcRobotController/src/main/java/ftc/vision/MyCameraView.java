package ftc.vision;

import android.content.Context;
import android.hardware.Camera;

import org.opencv.android.JavaCameraView;

/**
 * Created by student on 11/27/2017.
 */

public class MyCameraView extends JavaCameraView {
    Camera.Parameters params = null;

    public MyCameraView(Context context, int cameraId) {
        super(context, cameraId);
        params = mCamera.getParameters();
    }

    public void enableFlash(boolean flash) {
        params.setFlashMode(flash ? Camera.Parameters.FLASH_MODE_ON : Camera.Parameters.FLASH_MODE_OFF);
    }
}
