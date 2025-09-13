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

public class OuttakingState implements State {
    private final MyRobot robot;
    private final Task mainTask;

    public OuttakingState(MyRobot robot) {
        this.robot = robot;

        mainTask = new SequentialTask(robot,
                new WaitForDumpInputTask(robot),
                robot.claw.new MoveClawTask(robot, Claw.CLAW_OPEN_POSITION),
                new ParallelTask(robot, false,
                        robot.claw.new MoveWristTask(robot, Claw.WRIST_MAX_POSITION),
                        robot.xarm.new MoveExtensionTask(robot, X_arm.EXTENSION_OUTTAKING_POSITION),
                        robot.zarm.new MoveShoulderTask(robot, Z_arm.SHOULDER_OUTTAKING_POSITION)
                )
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

        return new IntakingState(robot);
    }

    @Override
    public String getName() {
        return "Outtaking";
    }

    private static class WaitForDumpInputTask extends Task {
        public WaitForDumpInputTask(MyRobot robot) {
            super(robot);
        }

        @Override
        protected void initialize(RobotContext ctx) {}

        @Override
        protected boolean run(RobotContext ctx) {
            return !((MyRobot) ctx).gamepad2.a;
        }
    }
}




