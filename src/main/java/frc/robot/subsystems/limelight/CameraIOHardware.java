package frc.robot.subsystems.limelight;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.utils.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class LimelightIOHardware implements LimelightIO{

    private final Drivetrain drivetrain = Robot.drivetrain;

    private final String limelightName;
    private final Pose3d robotToLimelight;
    private boolean doEstimation = false;
    private boolean goodEstimationFrame = true;

    private double runningTimer = 0;
    static boolean initialEstimationsComplete = false;
    static boolean doEstimationAll = true;
    private int[] tagFilter = new int[0];
    private boolean hasTarget = false;
    private LimelightHelpers.PoseEstimate estimatedPose;


    public LimelightIOHardware(String limelightName, Pose3d robotToLimelight) {
        this.limelightName = limelightName;
        this.robotToLimelight = robotToLimelight;
    }

    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        if (LimelightHelpers.getCameraPose3d_RobotSpace(limelightName) != robotToLimelight) {
            LimelightHelpers.setCameraPose_RobotSpace(limelightName, robotToLimelight.getX(), robotToLimelight.getY(), robotToLimelight.getZ(), Units.radiansToDegrees(robotToLimelight.getRotation().getX()), Units.radiansToDegrees(robotToLimelight.getRotation().getY()), Units.radiansToDegrees(robotToLimelight.getRotation().getZ()));
            inputs.robotToLimelight = robotToLimelight;
            inputs.limelightName = limelightName;
            return;
        }
        hasTarget = LimelightHelpers.getTV(limelightName);
        if (runningTimer < 15) {
            if (hasTarget) {
                estimatedPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
                if (LimelightHelpers.validPoseEstimate(estimatedPose)) {
                    drivetrain.addVisionMeasurement(estimatedPose.pose, Timer.getFPGATimestamp(), VecBuilder.fill(0.1, 0.1, 0.2));
                    runningTimer++;
                }
                return;
            } else {
                return;
            }
        } else if (runningTimer >= 15) {
            LimelightHelpers.SetIMUMode(limelightName, 1);
            LimelightHelpers.SetRobotOrientation(limelightName,
                    drivetrain.getPose().getRotation().getDegrees(),
                    0,
                    0,
                    0,
                    0,
                    0
            );
            initialEstimationsComplete = true;
            inputs.limelightGyroInitialized = true;
        }
        if (!inputs.limelightGyroInitialized && initialEstimationsComplete && LimelightHelpers.getTV(limelightName)) {
            LimelightHelpers.SetIMUMode(limelightName, 1);
            LimelightHelpers.SetRobotOrientation(limelightName,
                    drivetrain.getPose().getRotation().getDegrees(),
                    0,
                    0,
                    0,
                    0,
                    0
            );
            estimatedPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
            LimelightHelpers.SetIMUMode(limelightName, 2);
            inputs.limelightGyroInitialized = true;
        } else if (inputs.limelightGyroInitialized && initialEstimationsComplete && LimelightHelpers.getTV(limelightName)) {
             estimatedPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        }

        if (Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) > 720) {
            goodEstimationFrame = false;
        }
        if (estimatedPose.tagCount == 0) {
            goodEstimationFrame = false;
        }

        if (goodEstimationFrame && doEstimation && doEstimationAll) {
            drivetrain.addVisionMeasurement(estimatedPose.pose);
        }

        inputs.hasTarget = hasTarget;
        inputs.estimatedRobotPose = estimatedPose.pose;
        inputs.tagFilter = tagFilter;
        inputs.targetPose = LimelightHelpers.toPose2D(LimelightHelpers.getTargetPose_RobotSpace(limelightName));

    }

    @Override
    public void resetIMU() {
        doEstimation = false;

        LimelightHelpers.SetIMUMode(limelightName, 1);
        LimelightHelpers.SetRobotOrientation(limelightName,
                Robot.drivetrain.getPose().getRotation().getDegrees(),
                0,
                0,
                0,
                0,
                0
        );
        LimelightHelpers.SetIMUMode(limelightName, 2);

        doEstimation = true;
    }

    @Override
    public void setTagFilter(int[] ids) {
        tagFilter = ids;
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, ids);
    }

    @Override
    public void resetTagFilter() {
        tagFilter = new int[]{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, new int[]{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22});
    }

    @Override
    public double getXFromTag() {
        return LimelightHelpers.getTargetPose_RobotSpace(limelightName)[0];
    }

    @Override
    public double getYFromTag() {
        return LimelightHelpers.getTargetPose_RobotSpace(limelightName)[1];
    }

    @Override
    public double getZFromTag() {
        return LimelightHelpers.getTargetPose_RobotSpace(limelightName)[2];
    }

    @Override
    public boolean tagIsVisible() {
        return LimelightHelpers.getTV(limelightName);
    }

    @Override
    public double getTX() {
        return LimelightHelpers.getTX(limelightName);
    }

    @Override
    public double getTY() {
        return LimelightHelpers.getTY(limelightName);
    }

    @Override
    public void turnOffAprilTags() {
        doEstimation = false;
    }

    @Override
    public void turnOnAprilTags() {
        doEstimation = true;
    }

    @Override
    public void turnOffAllAprilTagsImplementation() {
        doEstimationAll = false;
    }

    @Override
    public void turnOnAllAprilTagsImplementation() {
        doEstimationAll = true;
    }

}
