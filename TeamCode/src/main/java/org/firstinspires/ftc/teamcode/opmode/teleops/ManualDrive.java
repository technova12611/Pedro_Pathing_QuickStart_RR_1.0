package org.firstinspires.ftc.teamcode.opmode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Memory;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Drone;
import org.firstinspires.ftc.teamcode.utils.software.ActionScheduler;
import org.firstinspires.ftc.teamcode.utils.hardware.GamePadController;
import org.firstinspires.ftc.teamcode.utils.software.SmartGameTimer;

@Config
@TeleOp(group = "Drive", name="Manual Drive")
public class ManualDrive extends LinearOpMode {
    public static double TURN_SPEED = 0.75;
    public static double DRIVE_SPEED = 1;
    public static double SLOW_TURN_SPEED = 0.3;
    public static double SLOW_DRIVE_SPEED = 0.3;

    public static double STRAFE_DISTANCE = 3.5;

    private SmartGameTimer smartGameTimer;
    private GamePadController g1, g2;
    private MecanumDrive drive;
    private ActionScheduler sched;
    private Intake intake;
    private Outtake outtake;
    private Hang hang;
    private Drone drone;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Init
        g1 = new GamePadController(gamepad1);
        g2 = new GamePadController(gamepad2);
        g1.update();
        g2.update();
        sched = new ActionScheduler();
        drive = new MecanumDrive(hardwareMap, Memory.LAST_POSE);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        hang = new Hang(hardwareMap);
        drone = new Drone(hardwareMap);

        if (Memory.RAN_AUTO) {
            smartGameTimer = new SmartGameTimer(true);
        } else { // No auto memory, pull in slides
            smartGameTimer = new SmartGameTimer(false);
            outtake.isAuto = false;
            outtake.prepTeleop();
        }

        intake.initialize(false);

        // Ready!
        telemetry.addLine("Ready!");
        telemetry.update();
        waitForStart();

        // Opmode start
        if (opModeIsActive()) {
            resetRuntime();
            g1.reset();
            g2.reset();

            // Finish pulling in slides
            if (!smartGameTimer.isNominal()) {
                outtake.finishPrepTeleop();
            }

            // Init opmodes
            outtake.initialize();
            drone.initialize();
        }

        // Main loop
        while (opModeIsActive()) {
            g1.update();
            g2.update();

            move();
            subsystemControls();

            drive.updatePoseEstimate();
            sched.update();
            outtake.update();
            intake.update();

            telemetry.addData("Time left", smartGameTimer.formattedString() + " (" + smartGameTimer.status() + ")");
            telemetry.update();
        }

        // On termination
        Memory.LAST_POSE = drive.pose;
    }

    private void move() {
        double speed = (1-Math.abs(g1.right_stick_x)) * (DRIVE_SPEED - SLOW_DRIVE_SPEED) + SLOW_DRIVE_SPEED;
        double input_x = Math.pow(-g1.left_stick_y, 3) * speed;
        double input_y = Math.pow(-g1.left_stick_x, 3) * speed;
        Vector2d input = new Vector2d(input_x, input_y);
        input = drive.pose.heading.times(input);

        double input_turn = Math.pow(g1.left_trigger - g1.right_trigger, 3) * TURN_SPEED;
        if (g1.leftBumper()) input_turn += SLOW_TURN_SPEED;
        if (g1.rightBumper()) input_turn -= SLOW_TURN_SPEED;

        drive.setDrivePowers(new PoseVelocity2d(input, input_turn));
    }

    boolean isSlideOut = false;
    boolean isStackIntakeOn = false;
    private void subsystemControls() {
        // Intake controls
        if (g1.aOnce()) {
            if (intake.intakeState == Intake.IntakeState.ON) {
                Actions.runBlocking(intake.intakeOff());
            } else {
                Actions.runBlocking(intake.intakeOn());
            }
        }
        if (g1.b()) {
            if (isSlideOut) {
                isSlideOut = false;
                Actions.runBlocking(outtake.retractOuttake());
            }
            Actions.runBlocking(intake.intakeReverse());
        }
        if (!g1.b() && intake.intakeState == Intake.IntakeState.REVERSING) {
            Actions.runBlocking(intake.intakeOff());
        }

        // Outtake controls
        if(g1.yLong()) {
            Actions.runBlocking(outtake.latchScore2());
        }
        else if (g1.yOnce()) {
            if (isSlideOut) {
                if(outtake.latchState == Outtake.OuttakeLatchState.LATCH_1) {
                    Actions.runBlocking(new SequentialAction(
                            outtake.latchScore2(),
                            new SleepAction(0.5)
                    ));
                }
                else {
                    Actions.runBlocking(new SequentialAction(
                            outtake.latchScore1(),
                            new SleepAction(0.5)
                    ));
                }
            } else {
                isSlideOut = true;
                Actions.runBlocking(new SequentialAction(
                        intake.intakeOff(),
                        outtake.prepareToSlide(),
                        outtake.extendOuttakeMid(),
                        new SleepAction(0.5),
                        outtake.prepareToScore()));
            }
        }

        if (Math.abs(g1.right_stick_y) > 0.25  && isSlideOut) {
            Actions.runBlocking(outtake.moveSliderBlocking(-g1.right_stick_y));
        }

        // Hang arms up/down
        if (g1.dpadUpOnce()) {
            Actions.runBlocking(hang.armsUp());
        }
        if (g1.dpadDownOnce()) {
            Actions.runBlocking(hang.armsDown());
        }

        // move left and right by one slot
        if (g1.dpadLeftOnce()) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(drive.pose.position.x, drive.pose.position.y + STRAFE_DISTANCE))
                    .build());
        }

        if (g1.dpadRightOnce()) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(drive.pose.position.x, drive.pose.position.y - STRAFE_DISTANCE))
                    .build());
        }

        // drone launch
        if (g1.backOnce()) {
            Actions.runBlocking( new SequentialAction(
                    intake.stackIntakeLinkageDown(),
                    new SleepAction(0.25),
                    drone.scoreDrone(),
                    new SleepAction(0.25),
                    intake.stackIntakeLinkageUp()));
        }

        // stack intake
        if(g1.start() && g1.xOnce()) {
            if(!isStackIntakeOn) {
                isStackIntakeOn = true;
                Actions.runBlocking(intake.intakeStackedPixels());
            }
        }
        else if(g1.xOnce()) {
            isStackIntakeOn = false;
            if(intake.stackIntakeState == Intake.StackIntakeState.DOWN) {
                Actions.runBlocking(new SequentialAction(
                        intake.prepareTeleOpsIntake(),
                        intake.intakeOff()));
            } else {
                Actions.runBlocking(
                        new SequentialAction(intake.prepareStackIntake(),
                        intake.intakeOn()));
            }
        }
    }
}

