package frc.systems;

import frc.robot.Robot;
import frc.utilities.Xbox;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Arm {

    VictorSPX left, right;
    double rightIntakeSpeed = 1;
    double rightOuttakeSpeed = -0.3;

    double leftIntakeSpeed = 1;
    double leftOuttakeSpeed = -0.3;

    boolean isCollecting = false;

    public Arm(int rightCANPort, int leftCANPort) {
        left = new VictorSPX(leftCANPort);
        right = new VictorSPX(rightCANPort);
    }

    public void intake() {
        left.set(ControlMode.PercentOutput, leftIntakeSpeed);
        right.set(ControlMode.PercentOutput, rightIntakeSpeed);
        isCollecting = true;
    }

    public void outtake() {
        left.set(ControlMode.PercentOutput, leftOuttakeSpeed);
        right.set(ControlMode.PercentOutput, rightOuttakeSpeed);
    }

    public void stop() {

        left.set(ControlMode.PercentOutput, 0);
        right.set(ControlMode.PercentOutput, 0);

        isCollecting = false;
    }

    public void performMainProcessing() {
        if (Robot.xboxJoystick.getRawButton(Xbox.RT)) {
            intake();
        } else if (Robot.xboxJoystick.getRawButton(Xbox.LT)) {
            outtake();
        } else {
            stop();
        }
        updateTelemetry();
    }

    public void updateTelemetry() {
        SmartDashboard.putBoolean("Collecting", isCollecting);
    }

}