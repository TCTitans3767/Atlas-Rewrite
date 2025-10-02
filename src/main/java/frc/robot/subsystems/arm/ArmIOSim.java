package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO {

    private final DCMotor armMotor = DCMotor.getKrakenX60(1);

    private final DCMotorSim armSim;

    private final PIDController armController = new PIDController(50, 0.0, 10);

    private boolean armClosedLoop = false;
    private double armSetpoint = 0.0;
    private double armAppliedVolts = 0.0;

    public ArmIOSim() {
        armSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(armMotor, 1, 1), armMotor);
        armController.enableContinuousInput(-1, 1);
        armController.setTolerance(0.01);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        if (armClosedLoop) {
            armAppliedVolts = armController.calculate(armSim.getAngularPositionRotations(), armSetpoint);
        } else {
            armController.reset();
        }

        armSim.setInputVoltage(armAppliedVolts);
        armSim.update(0.02);

        inputs.armPositionRotations = armSim.getAngularPositionRotations();
        inputs.armSetPoint = this.armSetpoint;
        inputs.armAppliedVolts = this.armAppliedVolts;
        inputs.armCurrent = armSim.getCurrentDrawAmps();
        inputs.armVelocityRotationsPerSec = Units.radiansToRotations(armSim.getAngularVelocityRadPerSec());
        inputs.motionMagicError = armController.getError();
        inputs.isArmAtSetpoint = armController.atSetpoint();
    }


    @Override
    public void setArmSpeed(double speed) {
        armAppliedVolts = speed * 12;
        armClosedLoop = false;
    }

    @Override
    public void setArmPosition(double position) {
        armSetpoint = position;
        armClosedLoop = true;
    }
}
