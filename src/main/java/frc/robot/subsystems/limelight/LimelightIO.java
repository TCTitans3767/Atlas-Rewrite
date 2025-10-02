package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.utils.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

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


    default void resetIMU() {
    }

    default void setTagFilter(int[] ids) {
    }

    default void resetTagFilter() {
    }

    default double getXFromTag() {
        return 0.0;
    }

    default double getYFromTag() {
        return 0.0;
    }

    default double getZFromTag() {
        return 0.0;
    }

    default boolean tagIsVisible() {
        return false;
    }

    default double getTX() {
        return 0.0;
    }

    default double getTY() {
        return 0;
    }

    default void turnOffAprilTags() {
    }

    default void turnOnAprilTags() {
    }

    default void turnOffAllAprilTagsImplementation() {
    }

    default void turnOnAllAprilTagsImplementation() {
    }

    default Rotation2d getTagAngleImplementation(int tagID) {
        return new Rotation2d();
    }

    default Pose2d getTagPoseImplementation(int tagID) {
        return new Pose2d();
    }

}
