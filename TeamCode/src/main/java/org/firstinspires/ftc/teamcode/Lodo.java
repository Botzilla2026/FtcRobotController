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
        /* roboCoords is the length thre array showing the coordinates on the odometry plane
        and the angle theta from the x-axis clockwise

        */

        double x0 = roboCoords[0];
        double y0 = roboCoords[1];
        double theta = roboCoords[2];

        // returns the coordinates of the ball
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);     // whichever pipeline index detects the ball
        limelight.start();

        // get camera input
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // calculate relative position
            double tx = result.getTx();
            double ty = result.getTy();

            // calculate delta x
            double delta_x = Math.tan((69.0 / 180.0) * Math.PI + Math.PI * (ty / 180.0));
            delta_x = 16.94 * delta_x;

            double delta_y;
            delta_y = delta_x * Math.tan(-(tx / 180.0) * Math.PI);

            // adjust for angle

            double xB = x0 + delta_x * Math.cos(theta) - delta_y * Math.sin(theta);
            double yB = y0 + delta_x * Math.sin(theta) + delta_y * Math.cos(theta);

            double heading = Math.atan2(delta_y, delta_x) + theta;
            return new double[]{xB, yB, Math.toDegrees(heading)};
        }

        return new double[]{};
    }
}
