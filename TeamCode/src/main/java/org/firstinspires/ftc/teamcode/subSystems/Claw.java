package org.firstinspires.ftc.teamcode.subSystems;

import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    public static final double WRIST_MAX_POS = 0.8;
    public static final double WRIST_MIN_POS = -0.5;
    public static final double CLAW_OPEN_POS = 0.5;
    public static final double CLAW_CLOSED_POS = 0.1;

    private final Servo clawServo;
    private final Servo wristServo;

    public Claw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
    }

    private double limit(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }



    public class MoveClawTask extends Task {
        private final double position;

        public MoveClawTask(RobotContext robotContext, double position) {
            super(robotContext);
            this.position = position;

        }

        @Override
        protected void initialize(RobotContext robotContext) {
            clawServo.setPosition(position);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return true;
        }
    }

    public class MoveWristTask extends Task {
        private final double position;

        public MoveWristTask(RobotContext robotContext, double position) {
            super(robotContext);
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

        public ManualWristTask(RobotContext robotContext, double increment) {
            super(robotContext);
            this.increment = increment;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            double newPos = wristServo.getPosition() + increment;
            newPos = limit(newPos, WRIST_MIN_POS, WRIST_MAX_POS);
            wristServo.setPosition(newPos);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return true;
        }
    }
}



