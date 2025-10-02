package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

public interface Limelight {

    default void resetIMU() {
    }

    default void setTagFilter(int[] ids) {
    }

    default void resetTagFilter() {
    }

    default double getXFromTag() {
        return 0;
    }

    default double getYFromTag() {
        return 0;
    }

    default double getZFromTag() {
        return 0;
    }

    default boolean tagIsVisible() {
        return false;
    }

    default void initialPoseEstimates() {
    }

    default double getTX() {
        return 0;
    }

    default double getTY() {
        return 0;
    }

    default void turnOffAprilTags() {
    }

    default void turnOnAprilTags() {
    }

    static void turnOffAllAprilTags() {
        Robot.limelight.turnOffAllAprilTagsImplementation();
    }

    abstract void turnOffAllAprilTagsImplementation();

    static void turnOnAllAprilTags() {
        Robot.limelight.turnOnAllAprilTagsImplementation();
    }

    abstract void turnOnAllAprilTagsImplementation();

    static Rotation2d getTagAngle(int tagID) {
        return Robot.limelight.getTagAngleImplementation(tagID);
    }

    abstract Rotation2d getTagAngleImplementation(int tagID);

    static Pose2d getTagPose(int tagID) {
        return Robot.limelight.getTagPoseImplementation(tagID);
    }

    abstract Pose2d getTagPoseImplementation(int tagID);

    static double getNearestReefAngle() {
        return Robot.limelight.getNearestReefAngleImplementation();
    }

    abstract double getNearestReefAngleImplementation();

    static int getNearestReefTag() {
        return Robot.limelight.getNearestReefTagImplementation();
    }

    abstract int getNearestReefTagImplementation();

    default void logCamera() {

    }
}
