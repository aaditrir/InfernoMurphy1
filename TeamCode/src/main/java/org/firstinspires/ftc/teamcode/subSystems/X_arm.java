package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class X_arm {
    private final Motor extensionMotor;
    private final PIDController extensionPID = new PIDController(0.01, 0, 0.005);

    public static final double EXTENSION_INTAKING_POSITION = 200;
    public static final double EXTENSION_OUTTAKING_POSITION = 1000;

    private double targetPosition = EXTENSION_INTAKING_POSITION;

    public X_arm(HardwareMap hardwareMap) {
        extensionMotor = new Motor(hardwareMap, "extensionMotor");
        extensionMotor.setRunMode(Motor.RunMode.RawPower);
        extensionMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extensionMotor.resetEncoder();
    }


    public void tickPID() {
        double currentPos = extensionMotor.getCurrentPosition();
        double power = extensionPID.calculate(currentPos, targetPosition);
        extensionMotor.set(power);
    }

    public void setPower(double power) {
        extensionMotor.set(power);
    }

    public double getCurrentPosition() {
        return extensionMotor.getCurrentPosition();
    }

    public class MoveExtensionTask extends Task {
        private final double taskTarget;

        public MoveExtensionTask(RobotContext robotContext, double taskTarget) {
            super(robotContext);
            this.taskTarget = taskTarget;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            targetPosition = taskTarget;
            extensionPID.setSetPoint(taskTarget);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            double error = Math.abs(extensionMotor.getCurrentPosition() - taskTarget);
            return error <= 10;
        }
    }
}

