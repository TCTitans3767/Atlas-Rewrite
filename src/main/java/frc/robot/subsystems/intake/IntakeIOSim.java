package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO{

    private final DCMotor intakeGearbox = DCMotor.getKrakenX60(1);
    private final DCMotorSim intakeMotorSim;

    private final DCMotor leftWheelGearbox = DCMotor.getKrakenX60(1);
    private final DCMotor rightWheelGearbox = DCMotor.getKrakenX60(1);

    private final DCMotorSim leftWheelSim;
    private final DCMotorSim rightWheelSim;

    private final PIDController intakeController = new PIDController(30, 0.0, 3);
    private final PIDController leftWheelController = new PIDController(30, 0.0, 3);
    private final PIDController rightWheelController = new PIDController(30, 0.0, 3);

    private boolean intakeClosedLoop = false;
    private boolean leftWheelClosedLoop = false;
    private boolean rightWheelClosedLoop = false;
    private double intakeAppliedVolts = 0.0;
    private double leftWheelAppliedVolts = 0.0;
    private double rightWheelAppliedVolts = 0.0;

    public IntakeIOSim() {
       intakeMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(intakeGearbox, 1, 1), intakeGearbox);
       intakeController.setTolerance(0.01);

       leftWheelSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(leftWheelGearbox, 1, 1), leftWheelGearbox);
       rightWheelSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(rightWheelGearbox, 1, 1), rightWheelGearbox);
       leftWheelController.setTolerance(0.01);
       rightWheelController.setTolerance(0.01);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        if (intakeClosedLoop) {
            intakeAppliedVolts = MathUtil.clamp(intakeController.calculate(intakeMotorSim.getAngularPositionRotations()), -12.0, 12.0);
        } else {
            intakeController.reset();
        }

        if (leftWheelClosedLoop) {
            leftWheelAppliedVolts = MathUtil.clamp(leftWheelController.calculate(Units.radiansToRotations(leftWheelSim.getAngularVelocityRadPerSec())), -12.0, 12.0);
        } else {
            leftWheelController.reset();
        }

        if (rightWheelClosedLoop) {
            rightWheelAppliedVolts = MathUtil.clamp(rightWheelController.calculate(Units.radiansToRotations(rightWheelSim.getAngularVelocityRadPerSec())), -12.0, 12.0);
        } else {
            rightWheelController.reset();
        }

        intakeMotorSim.setInputVoltage(intakeAppliedVolts);
        leftWheelSim.setInputVoltage(leftWheelAppliedVolts);
        rightWheelSim.setInputVoltage(rightWheelAppliedVolts);
        intakeMotorSim.update(0.02);
        leftWheelSim.update(0.02);
        rightWheelSim.update(0.02);

        inputs.intakeAppliedVolts = intakeAppliedVolts;
        inputs.intakeWheelLeftAppliedVolts = leftWheelAppliedVolts;
        inputs.intakeWheelRightAppliedVolts = rightWheelAppliedVolts;
        inputs.isIntakeAtSetpoint = intakeController.atSetpoint();
        inputs.isIntakeWheelLeftAtSetpoint = leftWheelController.atSetpoint();
        inputs.isIntakeWheelRightAtSetpoint = rightWheelController.atSetpoint();
        inputs.intakePosition = intakeMotorSim.getAngularPositionRad();
        inputs.intakeVelocityRotationsPerSec = Units.radiansToRotations(intakeMotorSim.getAngularVelocityRadPerSec());
        inputs.intakeCurrent = intakeMotorSim.getCurrentDrawAmps();
        inputs.intakeWheelLeftCurrent = leftWheelSim.getCurrentDrawAmps();
        inputs.intakeWheelRightCurrent = rightWheelSim.getCurrentDrawAmps();
        inputs.intakeWheelLeftVelocityRotationsPerSec = Units.radiansToRotations(leftWheelSim.getAngularVelocityRadPerSec());
        inputs.intakeWheelRightVelocityRotationsPerSec = Units.radiansToRotations(rightWheelSim.getAngularVelocityRadPerSec());
        inputs.intakeSetPosition = intakeController.getSetpoint();
        inputs.intakeWheelLeftWheelSetVelocity = leftWheelController.getSetpoint();
        inputs.intakeWheelRightWheelSetVelocity = rightWheelController.getSetpoint();
        inputs.intakeError = intakeController.getError();
    }

    @Override
    public void setIntakePosition(double position) {
        intakeClosedLoop = true;
        intakeController.setSetpoint(position);
    }

    @Override
    public void setIntakeWheelLeftVelocity(double velocity) {
        leftWheelClosedLoop = true;
        leftWheelController.setSetpoint(velocity);
    }

    @Override
    public void setIntakeWheelRightVelocity(double velocity) {
        rightWheelClosedLoop = true;
        rightWheelController.setSetpoint(velocity);
    }

    @Override
    public void setIntakeWheelSpeed(double speed) {
        rightWheelClosedLoop = true;
        leftWheelClosedLoop = true;
        rightWheelController.setSetpoint(speed);
        leftWheelController.setSetpoint(speed);
    }

    @Override
    public void setPivotSpeed(double speed) {
        intakeClosedLoop = false;
        intakeAppliedVolts = speed * 12;
    }
}
