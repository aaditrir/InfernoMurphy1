package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Z_arm {
    private final Motor slideMotor;
    private final PIDController slidePID = new PIDController(0.015, 0, 0.003);

    public static final double SHOULDER_INTAKING_POSITION = 0.0;
    public static final double SHOULDER_OUTTAKING_POSITION = 1000;

    private double targetPosition = SHOULDER_INTAKING_POSITION;

    public Z_arm(HardwareMap hardwareMap) {
        slideMotor = new Motor(hardwareMap, "extentionMotor");
        slideMotor.setRunMode(Motor.RunMode.RawPower);
        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideMotor.resetEncoder();
    }
    public double getCurrentPosition() {
        return slideMotor.getCurrentPosition();
    }


    public void tickPID() {
        double currentPos = slideMotor.getCurrentPosition();
        double power = slidePID.calculate(currentPos, targetPosition);
        slideMotor.set(power);
    }

    public void setPower(double power) {
        slideMotor.set(power);
    }

    public class MoveZArmTask extends Task {
        private final double taskTarget;

        public MoveZArmTask(RobotContext robotContext, double taskTarget) {
            super(robotContext);
            this.taskTarget = taskTarget;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            targetPosition = taskTarget;
            slidePID.setSetPoint(taskTarget);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            double error = Math.abs(slideMotor.getCurrentPosition() - taskTarget);
            return error <= 5;
        }
    }
}

