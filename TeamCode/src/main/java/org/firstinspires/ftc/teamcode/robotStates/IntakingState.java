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
    private final MyRobot robot;
    private final Task mainTask;

    public IntakingState(MyRobot robot) {
        this.robot = robot;

        mainTask = new SequentialTask(robot,
                new GrabTask(robot),
                new TransferTask(robot)
        );
    }

    @Override
    public State step() {
        Gamepad gp = robot.gamepad1;
        robot.drive.driveRobotCentric(gp.left_stick_x, gp.left_stick_y, gp.right_stick_x);

        // Update arm PID
        robot.xarm.tickPID();
        robot.zarm.tickPID();

        if (mainTask.step()) {
            return this;
        }

        return new OuttakingState(robot);
    }

    @Override
    public String getName() {
        return "Intaking";
    }


    private static class GrabTask extends ParallelTask {
        public GrabTask(MyRobot robot) {
            super(robot, true,
                    new WaitForTransferInputTask(robot)
            );
        }
    }

    private static class WaitForTransferInputTask extends Task {
        public WaitForTransferInputTask(MyRobot robot) {
            super(robot);
        }

        @Override
        protected void initialize(RobotContext ctx) {}

        @Override
        protected boolean run(RobotContext ctx) {
            return !((MyRobot) ctx).gamepad1.a; // Wait until gamepad1 'A' pressed
        }
    }

    private static class TransferTask extends SequentialTask {
        public TransferTask(MyRobot robot) {
            super(robot,
                    robot.claw.new MoveClawTask(robot, Claw.CLAW_CLOSED_POSITION),
                    new ParallelTask(robot, false,
                            robot.xarm.new MoveExtensionTask(robot, X_arm.EXTENSION_INTAKING_POSITION),
                            robot.zarm.new MoveShoulderTask(robot, Z_arm.SHOULDER_INTAKING_POSITION),
                            robot.claw.new MoveWristTask(robot, Claw.WRIST_MAX_POSITION)
                    )
            );
        }
    }
}



