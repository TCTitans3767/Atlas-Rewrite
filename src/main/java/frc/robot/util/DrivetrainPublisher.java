package frc.robot.util;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class DrivetrainPublisher {
    public static DrivetrainPublisher instance = new DrivetrainPublisher();

    private DoubleSupplier xVelocitySupplier;
    private DoubleSupplier yVelocitySupplier;
    private DoubleSupplier thetaVelocitySupplier;
    private BooleanSupplier acceptInputsSupplier;

    public DrivetrainPublisher() {

    }

    public void setXVelocitySupplier(DoubleSupplier xVelocitySupplier) {
        this.xVelocitySupplier = xVelocitySupplier;
    }

    public void setYVelocitySupplier(DoubleSupplier yVelocitySupplier) {
        this.yVelocitySupplier = yVelocitySupplier;
    }

    public void setThetaVelocitySupplier(DoubleSupplier thetaVelocitySupplier) {
        this.thetaVelocitySupplier = thetaVelocitySupplier;
    }

    public void setSuppliers(DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier, DoubleSupplier thetaVelocitySupplier) {
        this.xVelocitySupplier = xVelocitySupplier;
        this.yVelocitySupplier = yVelocitySupplier;
        this.thetaVelocitySupplier = thetaVelocitySupplier;
    }

    public void setAcceptInputsSupplier(BooleanSupplier acceptInputsSupplier) {
        this.acceptInputsSupplier = acceptInputsSupplier;
    }

    @FunctionalInterface
    public interface RecieveSupplierValues{
       void recieve(DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier, DoubleSupplier thetaVelocitySupplier, BooleanSupplier acceptInputsSupplier);
    }

    public void updateDrivetrain(RecieveSupplierValues driveFunction) {
        driveFunction.recieve(xVelocitySupplier, yVelocitySupplier, thetaVelocitySupplier, acceptInputsSupplier);
    }
}
