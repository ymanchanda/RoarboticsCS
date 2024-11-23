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
import org.firstinspires.ftc.teamcode.team.states.ITDClawArmStateMachine;
import org.firstinspires.ftc.teamcode.team.states.ITDClawStateMachine;
import org.firstinspires.ftc.teamcode.team.states.ITDLiftStateMachine;


@Autonomous(name = "Red Left Asc", group = "Pixel")
public class RedLeftITDASC extends LinearOpMode { //updated


    ITDBaseLACH drive;
    private static double dt;
    private static TimeProfiler updateRuntime;

    private static final double width = 16.375;
    private static final double length = 15.125;
    private static final double hook = 5d;
    private static final double hook2 = 4.7d;


    static final Vector2d path0 = new Vector2d(36 ,0); // blue left, not confirmed, maybe change y to a different location for space
    static final Vector2d path1 = new Vector2d(48 + (length/2), -48 - (width/2));
    static final Vector2d path2 = new Vector2d(24,0); //level ascent


    //ElapsedTime carouselTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    enum State {
        WAIT0,
        CLAWCLOSE,
        INITSTRAFE,
        LIFTUP,
        DEPOSIT,
        SAMPLES,
        TURNS,
        PICKUPARM,
        PICKUPCLAW,
        EXTENSION,
        MOVEARM,
        CLAWOPEN,
        MOVEARMBACK,
        LIFTDOWN,
        IDLE,
        PARK,
    }

    RedLeftITDASC.State currentState = RedLeftITDASC.State.IDLE;

    Pose2d startPoseRL = new Pose2d(72 + (length/2), -24 - (width/2)); //-72, 24 not confirmed
    //lift test needs to be done (values are estimated/inaccurate)
    private static final double HIGHBAR = 0d; //36 inches, 91.4 cm
    private static final double LOWBAR = 0d; //20 inches, 50.8 cm
    private static final double LOWBASKET =  0d; //25.75 inches, 65.4 cm
    private static final double HIGHBASKET = 0d; //43 inches, 109.2 cm

    boolean tf = false;

    int count = 0;

    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));

        drive = new ITDBaseLACH(hardwareMap);
        drive.setPoseEstimate(startPoseRL);
        drive.robot.getITDLiftSubsystem().getStateMachine().updateState(ITDLiftStateMachine.State.IDLE);
        drive.robot.getITDClawSubsystem().getStateMachine().updateState(ITDClawStateMachine.State.CLOSE);

        TrajectorySequence P0 = drive.trajectorySequenceBuilder(startPoseRL)
                .lineTo(path0)
                .build();

        TrajectorySequence P1 = drive.trajectorySequenceBuilder(P0.end())
                .lineTo(path1)
                .build();

        TrajectorySequence P2 = drive.trajectorySequenceBuilder(P1.end())
                .lineTo(path2)
                .build();

//        TrajectorySequence P3 = drive.trajectorySequenceBuilder(P2.end())
//                .lineTo(path3)
//                .build();
/*
        TrajectorySequence P4 = drive.trajectorySequenceBuilder(P3.end())
                .lineTo(path4)
                .build();

        TrajectorySequence P5 = drive.trajectorySequenceBuilder(P4.end())
                .lineTo(path5)
                .build();

        TrajectorySequence P6 = drive.trajectorySequenceBuilder(P5.end())
                .lineTo(path6)
                .build();
        */
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

        currentState = RedLeftITDASC.State.WAIT0;

        while (opModeIsActive() && !isStopRequested()) {

            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));

            switch (currentState) {

                case WAIT0:
                    if (waitTimer.milliseconds() >= 200)
                        currentState = State.CLAWCLOSE;
                    waitTimer.reset();
                    telemetry.addLine("in the wait0 state");
                    break;

                case CLAWCLOSE://ensures claw is closed with preloaded specimen
                    if(waitTimer.milliseconds() >= 250){
                        drive.robot.getITDClawSubsystem().getStateMachine().updateState(ITDClawStateMachine.State.CLOSE);
                        currentState = State.INITSTRAFE;
                        waitTimer.reset();
                    }
                    break;

                case INITSTRAFE: //goes to submerisble
                    if(!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(P0);
                        currentState = State.LIFTUP;
                        waitTimer.reset();
                    }
                    break;

                case LIFTUP: //goes to high bar
                    drive.robot.getITDLiftSubsystem().extend(HIGHBASKET);
                    if(!drive.isBusy() && waitTimer.milliseconds() >= 500){
                        drive.robot.getITDClawArmSubsystem().update(hook); //hook
                        currentState = State.DEPOSIT;
                        waitTimer.reset();
                    }
                    break;

                case DEPOSIT://deposits sample on to high bar
                    if(!drive.isBusy() && waitTimer.milliseconds() >= 300) {
                        drive.robot.getITDClawArmSubsystem().update(hook2); //hook
                        drive.robot.getITDClawSubsystem().getStateMachine().updateState(ITDClawStateMachine.State.OPEN);
                        currentState = State.SAMPLES;
                        waitTimer.reset();
                    }
                    break;

                case SAMPLES: //goes to set position to pick up all samples
                    if(!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(P1);
                        currentState = State.TURNS;
                    }
                    break;


                case TURNS://turn to sample 1, 2, & 3 (depends on count) and open claw
                    if(!drive.isBusy()) {
                        count++;
                        drive.robot.getITDClawSubsystem().getStateMachine().updateState(ITDClawStateMachine.State.OPEN);
                        if(count == 1)
                            drive.turn(Math.toRadians(-25));//edit once we test
                        if (count == 2)
                            drive.turn(Math.toRadians(25));
                        if (count == 3)
                            drive.turn(Math.toRadians(25));
                        if(count<=3) {
                            currentState = State.PICKUPARM;
                            waitTimer.reset();
                        }
                    }
                    break;

                case PICKUPARM: //moves arm to position for pick up and reopens claw
                    if(!drive.isBusy()) {
                        drive.robot.getITDClawSubsystem().getStateMachine().updateState(ITDClawStateMachine.State.OPEN);
                        drive.robot.getITDClawArmSubsystem().getStateMachine().updateState(ITDClawArmStateMachine.State.PICKUP);
                        drive.robot.getITDArmSubsystem().setDesiredSetpoint(.2d);//change to the actual value
                        if (count <= 3) {
                            currentState = State.PICKUPCLAW;
                            waitTimer.reset();
                        }
                    }
                    break;

                case PICKUPCLAW: // moves claw to position to close on sample and pickup the sample
                    if(!drive.isBusy()) {
                        drive.robot.getITDClawSubsystem().getStateMachine().updateState(ITDClawStateMachine.State.CLOSE);
                        if (count <= 3) {
                            currentState = State.EXTENSION;
                            waitTimer.reset();
                        }
                    }
                    break;

                case EXTENSION: //extend lift to high basket
                    if(!drive.isBusy()) {
                        drive.robot.getITDArmSubsystem().setDesiredSetpoint(5d); //change to the actual value for it to go up and deposit (fully extended)
                        drive.robot.getITDLiftSubsystem().extend(5d);
                        currentState = State.MOVEARM;
                        waitTimer.reset();
                    }
                    break;

                case MOVEARM: // move arm up to drop to the high basket
                    if(!drive.isBusy()) {
                        drive.robot.getITDClawArmSubsystem().getStateMachine().updateState(ITDClawArmStateMachine.State.DROP);
                        currentState = State.CLAWOPEN;
                        waitTimer.reset();
                    }
                    break;

                case CLAWOPEN://drop off in basket
                    if(!drive.isBusy() && waitTimer.milliseconds() >= 200) {
                        drive.robot.getITDClawSubsystem().getStateMachine().updateState(ITDClawStateMachine.State.OPEN);
                        currentState = State.MOVEARMBACK;
                        waitTimer.reset();
                    }
                    break;

                case MOVEARMBACK: // closes claw & brings arm back to original position
                    if(!drive.isBusy() && waitTimer.milliseconds() >= 150) {
                        drive.robot.getITDClawSubsystem().getStateMachine().updateState(ITDClawStateMachine.State.CLOSE);
                        drive.robot.getITDClawArmSubsystem().getStateMachine().updateState(ITDClawArmStateMachine.State.PICKUP);//is drop correct
                        currentState = State.TURNS;
                        waitTimer.reset();
                    }
                    break;

                case LIFTDOWN:// bring lift back down to intial position
                    if(!drive.isBusy() && count <= 3) {
                        drive.robot.getITDLiftSubsystem().retract();//is this correct...
                        currentState = State.PICKUPARM;
                    }
                    else if(!drive.isBusy() && count > 3)
                        currentState = State.PARK;
                    break;


                case PARK://parks in observation zone
                    drive.followTrajectorySequenceAsync(P2);
                    currentState = State.IDLE;
                    break;

                case IDLE://goes to idle
                    PoseStorage.currentPose = drive.getPoseEstimate();
                    drive.robot.getITDLiftSubsystem().extend(LOWBAR);
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