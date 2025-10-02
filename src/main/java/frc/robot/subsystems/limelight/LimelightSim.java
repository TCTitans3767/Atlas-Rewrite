package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSim extends SubsystemBase implements Limelight {

    public LimelightSim() {}

    @Override
    public void periodic() {
    }

    @Override
    public void turnOffAllAprilTagsImplementation() {

    }

    @Override
    public void turnOnAllAprilTagsImplementation() {

    }

    @Override
    public Rotation2d getTagAngleImplementation(int tagID) {
        return null;
    }

    @Override
    public Pose2d getTagPoseImplementation(int tagID) {
        return null;
    }

    @Override
    public double getNearestReefAngleImplementation() {
        return 0;
    }

    @Override
    public int getNearestReefTagImplementation() {
        return 0;
    }
}
