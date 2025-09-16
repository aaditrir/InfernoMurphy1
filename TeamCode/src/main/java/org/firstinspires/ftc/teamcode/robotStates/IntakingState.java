package org.firstinspires.ftc.teamcode.robotStates;

import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.states.State;
import com.jumpypants.murphy.tasks.ParallelTask;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.subSystems.Claw;
import org.firstinspires.ftc.teamcode.subSystems.X_arm;
import org.firstinspires.ftc.teamcode.subSystems.Z_arm;

public class IntakingState implements State {
    private final MyRobot robotContext;
    private final Task mainTask;

    public IntakingState(MyRobot robotContext) {
        this.robotContext = robotContext;

        mainTask = new SequentialTask(robotContext,
                new GrabTask(robotContext),
                new TransferTask(robotContext)
        );
    }

    @Override
    public State step() {
        Gamepad gamepad1 = robotContext.gamepad1;
        robotContext.drive.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        // Update arm PID
        robotContext.zarm.tickPID();
        robotContext.xarm.tickPID();

        if (mainTask.step()) {
            return this;
        }

        return new OuttakingState(robotContext);
    }

    @Override
    public String getName() {
        return "Intaki ng";
    }


    private static class GrabTask extends ParallelTask {
        public GrabTask(MyRobot robotContext) {
            super(robotContext, true,
                    new WaitForTransferInputTask(robotContext)
            );
        }
    }

    private static class WaitForTransferInputTask extends Task {
        public WaitForTransferInputTask(MyRobot robotContext) {
            super(robotContext);
        }

        @Override
        protected void initialize(RobotContext ctx) {}

        @Override
        protected boolean run(RobotContext ctx) {
            return !robotContext.gamepad1.a; // Wait until gamepad1 'A' pressed
        }
    }

    private static class TransferTask extends SequentialTask {
        public TransferTask(MyRobot robotContext) {
            super(robotContext,
                    robotContext.claw.new MoveClawTask(robotContext, Claw.CLAW_OPEN_POS),
                    new ParallelTask(robotContext, false,
                            robotContext.xarm.new MoveExtensionTask(robotContext, X_arm.EXTENSION_INTAKING_POSITION),
                            robotContext.zarm.new MoveZArmTask(robotContext, Z_arm.SHOULDER_INTAKING_POSITION),
                            robotContext.claw.new MoveWristTask(robotContext, Claw.WRIST_MAX_POS)
                    )
            );
        }
    }
}



