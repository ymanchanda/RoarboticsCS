package org.firstinspires.ftc.teamcode.team.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team.PoseStorage;
import org.firstinspires.ftc.teamcode.team.odometry.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.team.states.ITDLiftStateMachine;
import org.firstinspires.ftc.teamcode.team.states.ITDArmStateMachine;
import org.firstinspires.ftc.teamcode.team.states.ITDClawArmStateMachine;
import org.firstinspires.ftc.teamcode.team.states.ITDClawStateMachine;


@Autonomous(name = "Red Left", group = "Pixel")
public class RedLeftITD extends LinearOpMode {

    CSBaseLIO drive;
    private static double dt;
    private static TimeProfiler updateRuntime;

    enum State {
        WAIT0,
        CLAWCLOSE,
        INITSTRAFE,
        LIFTUP,
        FORWARD,
        PRELOAD,
        MOVEARM,
        CLAWOPEN,
        MOVEARMBACK,
        LIFTDOWN,
        IDLE,
        PARK,
        GRAB
    }

    private static final double width = 16.375;
    private static final double length = 15.125;

    private static final double HIGHBAR = 0d; //36 inches, 91.4 cm
    private static final double LOWBAR = 0d; //20 inches, 50.8 cm
    private static final double LOWBASKET =  0d; //25.75 inches, 65.4 cm
    private static final double HIGHBASKET = 0d; //43 inches, 109.2 cm

    boolean tf = false;

    int counter = 0;

    Pose2d startPoseRL = new Pose2d( 72 - (15.125/2), - 24 + (16.375/2)); // 72, -24 not confirmed
    static final Vector2d path1 = new Vector2d(48 - (15.125/2),-24); // blue left, not confirmed, maybe change y to a different location for space
    static final Vector2d path2 = new Vector2d(48 - (15.125/2), 12); // blue right, not confirmed, maybe change y to a different location for space
    static final Vector2d path3 = new Vector2d(24 + (15.125/2),-32); // red right, not confirmed, maybe change y to a different location for space
    static final Vector2d path4 = new Vector2d(0 + (15.125/2), -32); // red left, not confirmed, maybe change y to a different location for space
    //rotate 90 degrees right
    static final Vector2d path5 = new Vector2d(0, -12);
    //finished auto

    //ElapsedTime carouselTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    org.firstinspires.ftc.teamcode.team.auto.RedLeftITD.State currentState = org.firstinspires.ftc.teamcode.team.auto.RedLeftITD.State.IDLE;
    //State currentState = State.IDLE;




    //lift test needs to be done (values are estimated/inaccurate)



    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));

        drive = new CSBaseLIO(hardwareMap);
        drive.setPoseEstimate(startPoseRL);
        drive.robot.getLiftSubsystem().getStateMachine().updateState(ITDLiftStateMachine.State.IDLE);
        //drive.robot.getITDClawStateMachine().getStateMachine().updateState(ITDClawStateMachine.State.IDLE);

        TrajectorySequence P0 = drive.trajectorySequenceBuilder(startPoseRL)
                .lineTo(path1)
                .build();

        TrajectorySequence P1 = drive.trajectorySequenceBuilder(P0.end())
                .lineTo(path2)
                .build();

        TrajectorySequence P2 = drive.trajectorySequenceBuilder(P1.end())
                .lineTo(path3)
                .build();

        TrajectorySequence P3 = drive.trajectorySequenceBuilder(P2.end())
                .lineTo(path4)
                .build();

        TrajectorySequence P4 = drive.trajectorySequenceBuilder(P3.end())
                .lineTo(path5)
                .build();

        drive.getExpansionHubs().update(getDt());
        drive.robot.getLiftSubsystem().update(getDt());
        //drive.robot.getITDClawStateMachine().update(getDt());


        double t1 = waitTimer.milliseconds();

        double t2 = waitTimer.milliseconds();

        telemetry.addData("Initialize Time Seconds", (t2 - t1));
        telemetry.update();

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        currentState = org.firstinspires.ftc.teamcode.team.auto.RedLeftITD.State.WAIT0;

        while (opModeIsActive() && !isStopRequested()) {

            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));

            switch (currentState) {

                case WAIT0:
                    telemetry.addLine("in the wait0 state");
                    break;

                case CLAWCLOSE:
                    if (waitTimer.milliseconds() >= 1000) {
                        drive.robot.getITDClawSubsystem().getStateMachine().updateState(ITDClawStateMachine.CLOSE);

                    }
                    currentState = org.firstinspires.ftc.teamcode.team.auto.RedLeftITD.State.INITSTRAFE;
                    break;

                case INITSTRAFE:
                    currentState = org.firstinspires.ftc.teamcode.team.auto.RedLeftITD.State.LIFTUP;

                    break;

                case LIFTUP:
                    currentState = org.firstinspires.ftc.teamcode.team.auto.RedLeftITD.State.FORWARD;
                    break;

                case FORWARD:
                    currentState = org.firstinspires.ftc.teamcode.team.auto.RedLeftITD.State.PRELOAD;
                    break;

                case PRELOAD:
                    currentState = org.firstinspires.ftc.teamcode.team.auto.RedLeftITD.State.MOVEARM;
                    break;

                case MOVEARM:
                    currentState = org.firstinspires.ftc.teamcode.team.auto.RedLeftITD.State.CLAWOPEN;
                    break;

                case CLAWOPEN:
                    currentState = org.firstinspires.ftc.teamcode.team.auto.RedLeftITD.State.MOVEARMBACK;
                    break;

                case MOVEARMBACK:
                    currentState = org.firstinspires.ftc.teamcode.team.auto.RedLeftITD.State.LIFTDOWN;
                    break;

                case LIFTDOWN:
                    currentState = org.firstinspires.ftc.teamcode.team.auto.RedLeftITD.State.GRAB;
                    break;


                case GRAB:
                    currentState = State.PARK;
                    break;


                case PARK:
                    currentState = org.firstinspires.ftc.teamcode.team.auto.RedLeftITD.State.IDLE;
                    break;

                case IDLE:
                    PoseStorage.currentPose = drive.getPoseEstimate();
                    break;
            }

            drive.update();

            //The following code ensure state machine updates i.e. parallel execution with drivetrain
            drive.getExpansionHubs().update(getDt());
            drive.robot.getLiftSubsystem().update(getDt());
            drive.robot.getOuttakeSubsystem().update(getDt());
            drive.robot.getDroneSubsystem().update(getDt());
            telemetry.update();
        }


        drive.setMotorPowers(0.0,0.0,0.0,0.0);
    }
    public static TimeProfiler getUpdateRuntime() {
        return updateRuntime;
    }

    public static void setUpdateRuntime(TimeProfiler updaRuntime) {
        updateRuntime = updaRuntime;
    }

    public static double getDt() {
        return dt;
    }

    public static void setDt(double pdt) {
        dt = pdt;
    }
}








