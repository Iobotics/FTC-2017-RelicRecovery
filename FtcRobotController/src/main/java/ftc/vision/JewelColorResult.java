package ftc.vision;

import org.opencv.core.Scalar;

/**
 * Created by student on 11/6/2017.
 */

public class JewelColorResult {
    public JewelColorResult(JewelColor leftColor, JewelColor rightColor) {
        this.leftColor = leftColor;
        this.rightColor = rightColor;
    }

    public enum JewelColor {
        RED (ImageUtil.RED),
        BLUE (ImageUtil.BLUE),
        UNKNOWN (ImageUtil.BLACK);
        public final Scalar color;
        JewelColor(Scalar color) {
            this.color = color;
        }
    }

    private final JewelColor leftColor, rightColor;

    public JewelColor getLeftColor() {
        return leftColor;
    }

    public JewelColor getRightColor() {
        return rightColor;
    }

    @Override
    public String toString(){
        return leftColor + ", " + rightColor;
    }
}
