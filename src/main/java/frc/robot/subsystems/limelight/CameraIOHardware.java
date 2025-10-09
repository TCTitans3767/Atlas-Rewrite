package frc.robot.subsystems.limelight;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.utils.LimelightHelpers;
import limelight.Limelight;
import limelight.networktables.*;
import limelight.results.RawFiducial;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.DegreesPerSecond;

public class CameraIOHardware implements CameraIO {

    private final Drivetrain drivetrain = Robot.drivetrain;

    private final String limelightName;
    private final Pose3d robotToLimelight;
    private boolean doEstimation = false;
    private boolean goodEstimationFrame = true;

    private double runningTimer = 0;
    static boolean initialEstimationsComplete = false;
    static boolean doEstimationAll = true;
    private Double[] tagFilter = new Double[0];
    private boolean hasTarget = false;
    private LimelightHelpers.PoseEstimate estimatedPose;

    private final Limelight limelight;
    private final LimelightPoseEstimator limelightMegaTag2Estimator;
    private final LimelightPoseEstimator limelightMegaTag1Estimator;

    public CameraIOHardware(String limelightName, Pose3d robotToLimelight) {
        limelight = new Limelight(limelightName);
        limelight.getSettings().withCameraOffset(robotToLimelight).save();

        limelightMegaTag2Estimator = limelight.getPoseEstimator(true);
        limelightMegaTag1Estimator = limelight.getPoseEstimator(false);
        this.limelightName = limelightName;
        this.robotToLimelight = robotToLimelight;
    }

    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        if (runningTimer < 15 && doEstimationAll && doEstimation) {
            limelight.getSettings().withImuMode(LimelightSettings.ImuMode.SyncInternalImu).save();
            Optional<PoseEstimate> visionEstimate = limelightMegaTag1Estimator.getPoseEstimate();
            visionEstimate.ifPresent(poseEstimate -> {
                inputs.hasTarget = true;
                if ((Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) < 720) && (poseEstimate.tagCount > 0) && (poseEstimate.avgTagDist < 3.5) && (poseEstimate.getAvgTagAmbiguity() < 0.1)) {
                    drivetrain.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds, VecBuilder.fill(0.1, 0.1, 0.2));
                    runningTimer++;

                    inputs.estimatedRobotPose = poseEstimate.pose.toPose2d();
                    ArrayList<Pose2d> tagPoses = new ArrayList<>();
                    for (RawFiducial fiducial : poseEstimate.rawFiducials) {
                        tagPoses.add(getTagPoseImplementation(fiducial.id));
                    }
                    tagPoses.toArray(inputs.targetPoses);
                }
            });
        } else if (runningTimer >= 15 && doEstimationAll && doEstimation) {
            limelight.getSettings().withImuMode(LimelightSettings.ImuMode.SyncInternalImu).save();
            limelight.getSettings().withRobotOrientation(new Orientation3d(
                    drivetrain.getPigeon2().getRotation3d(),
                    new AngularVelocity3d(DegreesPerSecond.of(0), DegreesPerSecond.of(0), DegreesPerSecond.of(0))
            )).save();
            limelightMegaTag2Estimator.getPoseEstimate().ifPresent(poseEstimate -> {
                inputs.hasTarget = true;
                if ((Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) < 720) && (poseEstimate.tagCount > 0) && (poseEstimate.avgTagDist < 5) && (poseEstimate.getAvgTagAmbiguity() < 0.1)) {
                    inputs.validEstimationFrame = true;
                    drivetrain.addVisionMeasurement(poseEstimate.pose.toPose2d());

                    inputs.estimatedRobotPose = poseEstimate.pose.toPose2d();
                    ArrayList<Pose2d> tagPoses = new ArrayList<>();
                    for (RawFiducial fiducial : poseEstimate.rawFiducials) {
                        tagPoses.add(getTagPoseImplementation(fiducial.id));
                    }
                    tagPoses.toArray(inputs.targetPoses);
                }
            });
            limelight.getSettings().withImuMode(LimelightSettings.ImuMode.InternalImu).save();
            initialEstimationsComplete = true;
            inputs.limelightGyroInitialized = true;
        }
        if (inputs.limelightGyroInitialized && initialEstimationsComplete && doEstimationAll && doEstimation){
            limelightMegaTag2Estimator.getPoseEstimate().ifPresent(poseEstimate -> {
                inputs.hasTarget = true;
                if ((Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) < 720) && (poseEstimate.tagCount > 0) && (poseEstimate.avgTagDist < 5) && (poseEstimate.getAvgTagAmbiguity() < 0.1)) {
                    inputs.validEstimationFrame = true;
                    drivetrain.addVisionMeasurement(poseEstimate.pose.toPose2d());

                    inputs.estimatedRobotPose = poseEstimate.pose.toPose2d();
                    ArrayList<Pose2d> tagPoses = new ArrayList<>();
                    for (RawFiducial fiducial : poseEstimate.rawFiducials) {
                        tagPoses.add(getTagPoseImplementation(fiducial.id));
                    }
                    tagPoses.toArray(inputs.targetPoses);
                }
            });
        } else {
            inputs.hasTarget = false;
            inputs.validEstimationFrame = false;
            inputs.targetPoses = new Pose2d[0];
        }
    }

    @Override
    public void resetIMU() {
        doEstimation = false;

        limelight.getSettings().withImuMode(LimelightSettings.ImuMode.SyncInternalImu).save();
        limelight.getSettings().withRobotOrientation(new Orientation3d(
                drivetrain.getPigeon2().getRotation3d(),
                new AngularVelocity3d(DegreesPerSecond.of(0), DegreesPerSecond.of(0), DegreesPerSecond.of(0))
        )).save();

        doEstimation = true;
    }

    public void setTagFilter(Double[] ids) {
        tagFilter = ids;
        limelight.getSettings().withArilTagIdFilter(Arrays.asList(ids)).save();
    }

    @Override
    public void resetTagFilter() {
        tagFilter = new Double[]{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0};
        limelight.getSettings().withArilTagIdFilter(List.of(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0)).save();
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
