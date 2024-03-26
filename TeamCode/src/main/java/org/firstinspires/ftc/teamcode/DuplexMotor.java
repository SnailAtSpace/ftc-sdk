package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DuplexMotor implements DcMotorEx {
    DcMotorEx a, b;

    public DuplexMotor(DcMotorEx a, DcMotorEx b) {
        this.a = a;
        this.b = b;
    }

    @Override
    public void setMotorEnable() {
        a.setMotorEnable();
        b.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        a.setMotorDisable();
        b.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return a.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        a.setVelocity(angularRate, unit);
        b.setVelocity(angularRate, unit);
    }

    @Override
    public double getVelocity() {
        return a.getVelocity();
    }

    @Override
    public void setVelocity(double angularRate) {
        a.setVelocity(angularRate);
        b.setVelocity(angularRate);
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return a.getVelocity(unit);
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        return;
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        a.setPIDFCoefficients(mode, pidfCoefficients);
        b.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        a.setVelocityPIDFCoefficients(p, i, d, f);
        b.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        a.setPositionPIDFCoefficients(p);
        b.setPositionPIDFCoefficients(p);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return null;
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return a.getPIDFCoefficients(mode);
    }

    @Override
    public int getTargetPositionTolerance() {
        return a.getTargetPositionTolerance();
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        a.setTargetPositionTolerance(tolerance);
        b.setTargetPositionTolerance(tolerance);
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return a.getCurrent(unit) + b.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return a.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        a.setCurrentAlert(current, unit);
        b.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return a.isOverCurrent() || b.isOverCurrent();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return a.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        a.setMotorType(motorType);
        b.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return a.getController();
    }

    @Override
    public int getPortNumber() {
        return a.getPortNumber();
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return a.getZeroPowerBehavior();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        a.setZeroPowerBehavior(zeroPowerBehavior);
        b.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public void setPowerFloat() {
        a.setPowerFloat();
        b.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return a.getPowerFloat();
    }

    @Override
    public int getTargetPosition() {
        return a.getTargetPosition();
    }

    @Override
    public void setTargetPosition(int position) {
        a.setTargetPosition(position);
        b.setTargetPosition(position);
    }

    @Override
    public boolean isBusy() {
        return a.isBusy() || b.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return a.getCurrentPosition();
    }

    @Override
    public RunMode getMode() {
        return a.getMode();
    }

    @Override
    public void setMode(RunMode mode) {
        a.setMode(mode);
        b.setMode(mode);
    }

    @Override
    public Direction getDirection() {
        return a.getDirection();
    }

    @Override
    public void setDirection(Direction direction) {
        a.setDirection(direction);
        b.setDirection(direction != Direction.FORWARD ? Direction.REVERSE : Direction.FORWARD);
    }

    @Override
    public double getPower() {
        return a.getPower();
    }

    @Override
    public void setPower(double power) {
        a.setPower(power);
        b.setPower(power);
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {
        a.close();
        b.close();
    }
}
