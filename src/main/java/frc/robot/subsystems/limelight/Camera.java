package frc.robot.subsystems.limelight;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.utils.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class Limelight extends SubsystemBase {

    private final LimelightIO io;
    private final LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();

    private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public Limelight(LimelightIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Limelight" + inputs.limelightName, inputs);
    }

    public void resetIMU() {
        io.resetIMU();
    }

    public void setTagFilter(int[] ids) {
        io.setTagFilter(ids);
    }

    public void resetTagFilter() {
        io.resetTagFilter();
    }

    public double getXFromTag() {
        return io.getXFromTag();
    }

    public double getYFromTag() {
        return io.getYFromTag();
    }

    public double getZFromTag() {
        return io.getZFromTag();
    }

    public boolean tagIsVisible() {
        return io.tagIsVisible();
    }

    public double getTX() {
        return io.getTX();
    }

    public double getTY() {
        return io.getTY();
    }

    public void turnOffAprilTags() {
        io.turnOffAprilTags();
    }

    public void turnOnAprilTags() {
        io.turnOnAprilTags();
    }

    public void turnOffAllAprilTags() {
        io.turnOffAllAprilTagsImplementation();
    }

    public void turnOnAllAprilTags() {
        io.turnOnAllAprilTagsImplementation();
    }

    static public Rotation2d getTagAngle(int tagID) {
        return fieldLayout.getTagPose(tagID).get().toPose2d().getRotation();
    }

    static public Pose2d getTagPose(int tagID) {
        return fieldLayout.getTagPose(tagID).get().toPose2d();
    }

    public double getNearestReefAngle() {
        if (Robot.getAlliance() == DriverStation.Alliance.Blue) {
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

    public int getNearestReefTagImplementation() {
        if (Robot.getAlliance() == DriverStation.Alliance.Blue) {
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
}
