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
import org.firstinspires.ftc.teamcode.team.subsystems.ITDExpansionHubsLACH;


@Autonomous(name = "Blue Left", group = "Pixel")
public class BlueLeftITD extends LinearOpMode {

    ITDBaseLACH drive;
    private static double dt;
    private static TimeProfiler updateRuntime;

    static final Vector2d path0 = new Vector2d(-24 - (15.125/2),0); // blue left, not confirmed, maybe change y to a different location for space
    static final Vector2d path1 = new Vector2d(-24 - (15.125/2), 0); // blue right, not confirmed, maybe change y to a different location for space
    static final Vector2d path2 = new Vector2d(24 + (15.125/2),0); // red right, not confirmed, maybe change y to a different location for space
    static final Vector2d path3 = new Vector2d(24 + (15.125/2), 0); // red left, not confirmed, maybe change y to a different location for space
    static final Vector2d path4 = new Vector2d(50, 12);
    static final Vector2d path5 = new Vector2d(34, 12);
    static final Vector2d path6 = new Vector2d(12,12);

    //ElapsedTime carouselTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


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

    org.firstinspires.ftc.teamcode.team.auto.BlueLeftITD.State currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftITD.State.IDLE;

    private static final double width = 16.375;
    private static final double length = 15.125;

    Pose2d startPoseRL = new Pose2d( 72 - (15.125/2), - 24 + (16.375/2)); // 72, -24 not confirmed
    Pose2d startPoseRR = new Pose2d(72 - (15.125/2), 24 - (16.375/2)); //72, 24 not confirmed
    Pose2d startPoseBR = new Pose2d(- 72 + (15.125/2), - 24 + (16.375/2)); //-72, -24 not confirmed
    Pose2d startPoseBL = new Pose2d(- 72 + (15.125/2), 24 - (16.375/2)); //-72, 24 not confirmed
    //lift test needs to be done (values are estimated/inaccurate)
    private static final double HIGHBAR = 0d; //36 inches, 91.4 cm
    private static final double LOWBAR = 0d; //20 inches, 50.8 cm
    private static final double LOWBASKET =  0d; //25.75 inches, 65.4 cm
    private static final double HIGHBASKET = 0d; //43 inches, 109.2 cm

    boolean tf = false;

    int counter = 0;

    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));

        drive = new ITDBaseLACH(hardwareMap);
        drive.setPoseEstimate(startPoseBL);
        drive.robot.getITDLiftSubsystem().getStateMachine().updateState(ITDLiftStateMachine.State.IDLE);
        drive.robot.getITDClawSubsystem().getStateMachine().updateState(ITDClawStateMachine.State.CLOSE);

        TrajectorySequence P0 = drive.trajectorySequenceBuilder(startPoseBL)
                .lineTo(path0)
                .build();

        TrajectorySequence P1 = drive.trajectorySequenceBuilder(P0.end())
                .lineTo(path1)
                .build();

        TrajectorySequence P2 = drive.trajectorySequenceBuilder(P1.end())
                .lineTo(path2)
                .build();

        TrajectorySequence P3 = drive.trajectorySequenceBuilder(P2.end())
                .lineTo(path3)
                .build();

        TrajectorySequence P4 = drive.trajectorySequenceBuilder(P3.end())
                .lineTo(path4)
                .build();

        TrajectorySequence P5 = drive.trajectorySequenceBuilder(P4.end())
                .lineTo(path5)
                .build();

        TrajectorySequence P6 = drive.trajectorySequenceBuilder(P5.end())
                .lineTo(path6)
                .build();

        //drive.getITDExpansionHubsLACH().update(getDt());
        drive.robot.getITDLiftSubsystem().update(getDt());
        //drive.robot.getITDClawStateMachine().update(getDt());


        double t1 = waitTimer.milliseconds();

        double t2 = waitTimer.milliseconds();

        telemetry.addData("Initialize Time Seconds", (t2 - t1));
        telemetry.update();

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftITD.State.WAIT0;

        while (opModeIsActive() && !isStopRequested()) {

            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));

            switch (currentState) {

                case WAIT0:
                    if (waitTimer.milliseconds() >= 1000)
                        currentState = State.CLAWCLOSE;
                    waitTimer.reset();
                    telemetry.addLine("in the wait0 state");
                    break;

                case CLAWCLOSE:
                    if(waitTimer.milliseconds() >= 1000){
                        drive.robot.getITDClawSubsystem().getStateMachine().updateState(ITDClawStateMachine.State.CLOSE);
                        currentState = State.INITSTRAFE;
                        waitTimer.reset();
                    }
                    break;

                case INITSTRAFE:
                    if(!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(P0);
                        currentState = State.LIFTUP;
                        waitTimer.reset();
                    }
                    break;

                case LIFTUP:
                    currentState = State.FORWARD;
                    break;

                case FORWARD:
                    currentState = State.PRELOAD;
                    break;

                case PRELOAD:
                    currentState = State.MOVEARM;
                    break;

                case MOVEARM:
                    currentState = State.CLAWOPEN;
                    break;

                case CLAWOPEN:
                    currentState = State.MOVEARMBACK;
                    break;

                case MOVEARMBACK:
                    currentState = State.LIFTDOWN;
                    break;

                case LIFTDOWN:
                    currentState = State.GRAB;
                    break;


                case GRAB:
                    currentState = State.PARK;
                    break;


                case PARK:
                    currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftITD.State.IDLE;
                    break;

                case IDLE:
                    PoseStorage.currentPose = drive.getPoseEstimate();
                    break;
            }

            drive.update();

            //The following code ensure state machine updates i.e. parallel execution with drivetrain
           // drive.getITDExpansionHubsLACH().update(getDt());
            drive.robot.getITDLiftSubsystem().update(getDt());
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








