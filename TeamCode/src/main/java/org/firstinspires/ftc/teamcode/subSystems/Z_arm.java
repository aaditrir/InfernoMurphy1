package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Z_arm {
    private final Motor shoulderMotor;
    private final PIDController shoulderPID = new PIDController(0.015, 0, 0.003);

    public static final double SHOULDER_INTAKING_POSITION = 0.0;
    public static final double SHOULDER_OUTTAKING_POSITION = 1000;

    private double targetPosition = SHOULDER_INTAKING_POSITION;

    public Z_arm(HardwareMap hardwareMap) {
        shoulderMotor = new Motor(hardwareMap, "shoulderMotor");
        shoulderMotor.setRunMode(Motor.RunMode.RawPower);
        shoulderMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shoulderMotor.resetEncoder();
    }
    public double getCurrentPosition() {
        return shoulderMotor.getCurrentPosition();
    }


    public void tickPID() {
        double currentPos = shoulderMotor.getCurrentPosition();
        double power = shoulderPID.calculate(currentPos, targetPosition);
        shoulderMotor.set(power);
    }

    public void setPower(double power) {
        shoulderMotor.set(power);
    }

    public class MoveShoulderTask extends Task {
        private final double taskTarget;

        public MoveShoulderTask(RobotContext robotContext, double taskTarget) {
            super(robotContext);
            this.taskTarget = taskTarget;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            targetPosition = taskTarget;
            shoulderPID.setSetPoint(taskTarget);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            double error = Math.abs(shoulderMotor.getCurrentPosition() - taskTarget);
            return error <= 5;
        }
    }
}

