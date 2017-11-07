package ftc.vision;

import org.opencv.core.Scalar;

/**
 * Created by student on 11/6/2017.
 */

public class BeaconColorResult {
    public BeaconColorResult(BeaconColor leftColor, BeaconColor rightColor) {
        this.leftColor = leftColor;
        this.rightColor = rightColor;
    }

    public enum BeaconColor {
        RED (ImageUtil.RED),
        GREEN (ImageUtil.GREEN),
        BLUE (ImageUtil.BLUE),
        UNKNOWN (ImageUtil.BLACK);
        public final Scalar color;
        BeaconColor(Scalar color) {
            this.color = color;
        }
    }

    private final BeaconColor leftColor, rightColor;

    public BeaconColor getLeftColor() {
        return leftColor;
    }

    public BeaconColor getRightColor() {
        return rightColor;
    }

    @Override
    public String toString(){
        return leftColor + ", " + rightColor;
    }
}
