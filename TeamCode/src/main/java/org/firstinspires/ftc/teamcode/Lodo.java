package org.firstinspires.ftc.teamcode;
// Limelight camera function for

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.Math;

// to run in the main function Lodo lodo = new Lodo(hardwareMap);  hardwareMap exists here, because this IS the OpMode
//double[] ballCoords = lodo.getBall(roboCoords);

public class Lodo {
    private Limelight3A limelight;
    public double[] getBall(double[] roboCoords, HardwareMap hardwareMap){
        // roboCoords is the length two array showing the coordinates on the odometry plane
        // returns the coordinates of the ball
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // get camera input
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy();

            // calculate delta x
            double delta_x = Math.tan((69.0 / 180.0) * Math.PI + Math.PI * (ty / 180.0));
            delta_x = 16.94 * delta_x;

            double delta_y;
            delta_y = delta_x * Math.tan(-(tx / 180.0) * Math.PI);

            return new double[]{roboCoords[0] + delta_x, roboCoords[1] + delta_y};
        }

        return new double[]{};
    }
}
