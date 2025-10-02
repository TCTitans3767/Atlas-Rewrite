package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

import java.util.ArrayList;

public interface LimelightIO {

    public static enum limelightMode {
        megaTag2, megaTag1, objectDetection
    }

    @AutoLog
    public static class LimelightIOInputs {
        public String limelightName = "";
        public Pose3d robotToLimelight = new Pose3d();
        public boolean limelightGyroInitialized = false;
        public boolean hasTarget = false;
//        public double targetArea = 0.0;
//        public double targetSkew = 0.0;
//        public double targetLatency = 0.0;
//        public double targetShortestSide = 0.0;
//        public double targetLongestSide = 0.0;
//        public double targetHorizontalAngle = 0.0;
//        public double targetVerticalAngle = 0.0;
        public int[] tagFilter = new int[0];
        public Pose2d targetPose = new Pose2d();
        public Pose2d estimatedRobotPose = new Pose2d();
    }

    default void updateInputs(LimelightIOInputs inputs) {};
}
