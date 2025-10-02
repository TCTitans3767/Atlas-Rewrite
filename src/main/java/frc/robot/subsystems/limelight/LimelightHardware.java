package frc.robot.subsystems.limelight;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import org.littletonrobotics.junction.Logger;

public class LimelightHardware extends SubsystemBase implements Limelight {

    private int currentIMUMode = 1;

    private final String limelightName;
    private final Pose3d robotToLimelight;

    private final Drivetrain drivetrain = Robot.drivetrain;

    private boolean doEstimation = false;
    private static boolean doEstimationAll = true;
    private boolean goodEstimationFrame = true;
    private boolean robotToLimelightSet = true;
    private static AprilTagFieldLayout fieldLayout;

    private int startupEstimations = 0;

    PoseEstimate estimatedPose;
    double runningTimerStart = 0;

    public LimelightHardware(String limelightName, Pose3d robotToLimelight) {
        System.out.println("started limelight initialization");

        try {
            fieldLayout = new AprilTagFieldLayout(".\\src\\main\\deploy\\AprilTagCalibrations\\field_calibration.json");
            System.err.println("successfully read calibrated field layout!");
        } catch (Exception e) {
            System.err.println("failed to read calibrated field layout! \n Falling back to 2025 Reefscape Welded!");
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        }

        this.limelightName = limelightName;
        this.robotToLimelight = robotToLimelight;

        LimelightHelpers.setCameraPose_RobotSpace(
                limelightName,
                robotToLimelight.getX(),
                robotToLimelight.getY(),
                robotToLimelight.getZ(),
                Units.radiansToDegrees(robotToLimelight.getRotation().getX()),
                Units.radiansToDegrees(robotToLimelight.getRotation().getY()),
                Units.radiansToDegrees(robotToLimelight.getRotation().getZ())
        );
        robotToLimelightSet = true;

        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.3, 0.3, 999999));

        System.out.println("finished limelight initialization");

        runningTimerStart = Timer.getFPGATimestamp();

    }

    @Override
    public void periodic() {

        goodEstimationFrame = true;

        if (Timer.getFPGATimestamp() - runningTimerStart >= 10) {
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
        } else {
            LimelightHelpers.SetIMUMode(limelightName, 2);
            estimatedPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        }

        if (estimatedPose == null) {
            return;
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

        // SmartDashboard.putNumber(limelightName + "/Target Pose X", LimelightHelpers.getTargetPose_RobotSpace(limelightName)[0]);
        // SmartDashboard.putNumber(limelightName + "/Target Pose Y", LimelightHelpers.getTargetPose_RobotSpace(limelightName)[1]);
        // SmartDashboard.putNumber(limelightName + "/Target Pose Z", LimelightHelpers.getTargetPose_RobotSpace(limelightName)[2]);

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
        Logger.recordOutput(limelightName + "/Tag Filter", ids);
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, ids);
    }

    @Override
    public void resetTagFilter() {
        Logger.recordOutput(limelightName + "/Tag Filter", new int[]{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22});
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
    public void initialPoseEstimates() {
        if (robotToLimelightSet) {
            doEstimation = false;
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.1, 0.1, 0));
            int numTries = 0;
            while (startupEstimations < 10 && numTries < 40) {
                PoseEstimate result = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
                if (result == null) {
                    continue;
                }
                if (result.tagCount == 1) {
                    if (result.rawFiducials[0].ambiguity > 0.4) {
                        numTries++;
                        continue;
                    }
                    if (result.rawFiducials[0].distToCamera > 3) {
                        numTries++;
                        continue;
                    }
                }

                if (result.tagCount == 0) {
                    numTries++;
                    continue;
                }

                drivetrain.addVisionMeasurement(result.pose);

                startupEstimations++;
            }

            // resetIMU(drivetrain.getRotation3d());
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.3, 0.3, 99999));
            turnOnAprilTags();
        }
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

    @Override
    public Rotation2d getTagAngleImplementation(int tagID) {
        return fieldLayout.getTagPose(tagID).get().toPose2d().getRotation();
    }

    @Override
    public Pose2d getTagPoseImplementation(int tagID) {
        return fieldLayout.getTagPose(tagID).get().toPose2d();
    }

    @Override
    public double getNearestReefAngleImplementation() {
        if (Robot.getAlliance() == Alliance.Blue) {
            List<Integer> nearestTags = Arrays.asList(Constants.ReefTagIDs.blue);

            nearestTags.sort(new Comparator<Integer>() {

                Drivetrain drivetrain = Robot.drivetrain;

                @Override
                public int compare(Integer tag1, Integer tag2) {
                    double distanceToTag1 = Limelight.getTagPose(tag1).getTranslation().getDistance(drivetrain.getPose().getTranslation());
                    double distanceToTag2 = Limelight.getTagPose(tag2).getTranslation().getDistance(drivetrain.getPose().getTranslation());

                    return (int) Math.floor(distanceToTag1 - distanceToTag2);
                }
            });

            return Limelight.getTagAngle(nearestTags.get(0)).plus(Rotation2d.fromDegrees(180)).getDegrees();

        } else {
            List<Integer> nearestTags = Arrays.asList(Constants.ReefTagIDs.red);

            nearestTags.sort(new Comparator<Integer>() {

                Drivetrain drivetrain = Robot.drivetrain;

                @Override
                public int compare(Integer tag1, Integer tag2) {
                    double distanceToTag1 = Limelight.getTagPose(tag1).getTranslation().getDistance(drivetrain.getPose().getTranslation());
                    double distanceToTag2 = Limelight.getTagPose(tag2).getTranslation().getDistance(drivetrain.getPose().getTranslation());

                    return (int) Math.floor(distanceToTag1 - distanceToTag2);
                }
            });

            return Limelight.getTagAngle(nearestTags.get(0)).plus(Rotation2d.fromDegrees(180)).getDegrees();
        }
    }

    @Override
    public int getNearestReefTagImplementation() {
        if (Robot.getAlliance() == Alliance.Blue) {
            List<Integer> nearestTags = Arrays.asList(Constants.ReefTagIDs.blue);

            nearestTags.sort(new Comparator<Integer>() {

                Drivetrain drivetrain = Robot.drivetrain;

                @Override
                public int compare(Integer tag1, Integer tag2) {
                    double distanceToTag1 = Limelight.getTagPose(tag1).getTranslation().getDistance(drivetrain.getPose().getTranslation());
                    double distanceToTag2 = Limelight.getTagPose(tag2).getTranslation().getDistance(drivetrain.getPose().getTranslation());

                    return (int) Math.floor(distanceToTag1 - distanceToTag2);
                }
            });

            return nearestTags.get(0);

        } else {
            List<Integer> nearestTags = Arrays.asList(Constants.ReefTagIDs.red);

            nearestTags.sort(new Comparator<Integer>() {

                Drivetrain drivetrain = Robot.drivetrain;

                @Override
                public int compare(Integer tag1, Integer tag2) {
                    double distanceToTag1 = Limelight.getTagPose(tag1).getTranslation().getDistance(drivetrain.getPose().getTranslation());
                    double distanceToTag2 = Limelight.getTagPose(tag2).getTranslation().getDistance(drivetrain.getPose().getTranslation());

                    return (int) Math.floor(distanceToTag1 - distanceToTag2);
                }
            });

            return nearestTags.get(0);
        }
    }

    @Override
    public void logCamera() {

    }

}