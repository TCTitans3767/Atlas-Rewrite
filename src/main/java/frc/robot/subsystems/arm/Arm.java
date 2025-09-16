package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public Arm(ArmIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

    public void setPosition(double position) {
        io.setArmPosition(position);
    }

    public void setSpeed(double speed) {
        io.setArmSpeed(speed);
    }

    public boolean isAtPosition() {
        return inputs.isArmAtSetpoint;
    }

    public double getMotionMagicError() {
        return inputs.motionMagicError;
    }

    public double getPosition() {
        return inputs.armPositionRotations;
    }

    public boolean isNear(double position) {
        return MathUtil.isNear(position, inputs.armPositionRotations, Constants.Arm.errorTolerance);
    }
}
