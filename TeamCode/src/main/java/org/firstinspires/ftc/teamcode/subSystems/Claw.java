package org.firstinspires.ftc.teamcode.subSystems;

import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    public static final double CLAW_OPEN_POSITION = 0.7;
    public static final double CLAW_CLOSED_POSITION = 0.45;
    public static final double WRIST_MAX_POSITION = 0.7;
    public static final double WRIST_MIN_POSITION = 0.2;

    private final Servo clawServo;
    private final Servo wristServo;

    public Claw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
    }

    public class MoveClawTask extends Task {
        private final double position;

        public MoveClawTask(RobotContext robot, double position) {
            super(robot);
            this.position = position;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            clawServo.setPosition(position);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            // simple instant task
            return true;
        }
    }

    public class MoveWristTask extends Task {
        private final double position;

        public MoveWristTask(RobotContext robot, double position) {
            super(robot);
            this.position = position;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            wristServo.setPosition(position);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return true;
        }
    }

    public class ManualWristTask extends Task {
        private final double increment;

        public ManualWristTask(RobotContext robot, double increment) {
            super(robot);
            this.increment = increment;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            double newPos = wristServo.getPosition() + increment;
            newPos = Math.max(WRIST_MIN_POSITION, Math.min(WRIST_MAX_POSITION, newPos));
            wristServo.setPosition(newPos);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return true;
        }
    }
}


