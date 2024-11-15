package org.firstinspires.ftc.teamcode.team.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team.CSVP;
import org.firstinspires.ftc.teamcode.team.PoseStorage;
import org.firstinspires.ftc.teamcode.team.odometry.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.team.states.DroneStateMachine;
import org.firstinspires.ftc.teamcode.team.states.LiftStateMachine;
import org.firstinspires.ftc.teamcode.team.states.OuttakeStateMachine;
package org.firstinspires.ftc.teamcode.team.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team.CSVP;
import org.firstinspires.ftc.teamcode.team.PoseStorage;
import org.firstinspires.ftc.teamcode.team.odometry.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.team.states.OuttakeStateMachine;
import org.firstinspires.ftc.teamcode.team.states.DroneStateMachine;
import org.firstinspires.ftc.teamcode.team.states.LiftStateMachine;

@Autonomous(name = "Blue Right", group = "Pixel")
public class BlueRightITD extends LinearOpMode {

    CSBaseLIO drive;
    private static double dt;
    private static TimeProfiler updateRuntime;

    static final Vector2d Traj0 = new Vector2d(34,65.5);
    static final Vector2d Traj1 = new Vector2d(34, 12);
    static final Vector2d Traj2 = new Vector2d(20.5,12.5);
    static final Vector2d Traj3 = new Vector2d(54, 13);
    static final Vector2d Location1 = new Vector2d(50, 12);
    static final Vector2d Location2 = new Vector2d(34, 12);
    static final Vector2d Location3 = new Vector2d(12,12);

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
        TODROP,
        TOSTACK,
        IDLE,
        PARK,
        GRAB
    }
    State currentState = State.IDLE;


    private static final double width = 16.375;
    private static final double length = 15.125;

    Pose2d startPose = new Pose2d(0, -72 + length/2);
    //lift test needs to be done (values are estimated/inaccurate)
    private static final double HIGHBAR = 24d; //36 inches, 91.4 cm
    private static final double LOWBAR = 23.5d; //20 inches, 50.8 cm
    private static final double LOWBASKET = 0d; //25.75 inches, 65.4 cm
    private static final double HIGHBASKET = 0d; //43 inches, 109.2 cm




    org.firstinspires.ftc.teamcode.team.CSVP CSVP;
    boolean hasCVInit = false;
    int placement = 1;
    float confidence = 0;

    boolean tf = false;

    int counter = 0;

    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));

        drive = new CSBaseLIO(hardwareMap);
        drive.setPoseEstimate(startPose);
        drive.robot.getLiftSubsystem().getStateMachine().updateState(LiftStateMachine.State.IDLE);
        drive.robot.getDroneSubsystem().getStateMachine().updateState(DroneStateMachine.State.OPEN);
        drive.robot.getOuttakeSubsystem().getStateMachine().updateState(OuttakeStateMachine.State.FORWARD);
        //drive.robot.getCappingArmSubsystem().getStateMachine().updateState(ArmStateMachine.State.REST);

        TrajectorySequence traj0 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(Traj0)
                .build();

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(traj0.end())
                .lineTo(Traj1)
                .turn(Math.toRadians(93.5))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineTo(Traj2)
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .lineTo(Traj3)
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .lineTo(Traj2)
                .build();

        TrajectorySequence location1 = drive.trajectorySequenceBuilder(traj4.end())
                .lineTo(Location1)
                .build();

        TrajectorySequence location2 = drive.trajectorySequenceBuilder(traj4.end())
                .lineTo(Location2)
                .build();

        TrajectorySequence location3 = drive.trajectorySequenceBuilder(traj4.end())
                .lineTo(Location3)
                .build();

        drive.getExpansionHubs().update(getDt());

        drive.robot.getLiftSubsystem().update(getDt());
        drive.robot.getOuttakeSubsystem().update(getDt());
        drive.robot.getDroneSubsystem().update(getDt());

        double t1 = waitTimer.milliseconds();

        CSVP = new CSVP();
        CSVP.initTfod(hardwareMap);

        double t2 = waitTimer.milliseconds();

        telemetry.addData("Initialize Time Seconds", (t2 - t1));
        telemetry.update();

        int detectCounter = 0;
        //double confidence = 0;
        //String label = "NONE";
        int oldRecog = 0;
        int recog;

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;



        currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftFTC.State.WAIT0;

        while (opModeIsActive() && !isStopRequested()) {

            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));

            switch (currentState) {

                case WAIT0:
                    telemetry.addLine("in the wait0 state");
                    recog = CSVP.detect();
                    detectCounter++;
                    if (recog != 0){//edited
                        if(oldRecog != 0) {//edited
                            if (CSVP.detect() == recog){
                                //confidence = recog.getConfidence();
                                //label = recog.getLabel(); //object name (redObject...)
                                oldRecog = recog;
                                detectCounter++;
                                //add if location detected is left center or right (1,2,3) then itll move accordingly
                            }
                        }
                        else{
                            oldRecog = recog;
                        }
                    }
                    else {
                        telemetry.addLine("NULL");
                    }
                    if (waitTimer.milliseconds() > 3000 && confidence > 0.01){
                        currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftFTC.State.CLAWCLOSE;
                        placement = recog;
                    }
                    break;

                case CLAWCLOSE:
                    telemetry.addData("Location: ", placement); // added to return if left center or right is detected
                    //telemetry.addData("Label: ", label);
                    //telemetry.addData("Confidence: ", confidence);
                    telemetry.addData("#: ", detectCounter);
                    if(waitTimer.milliseconds() >= 1000){
                        drive.robot.getDroneSubsystem().getStateMachine().updateState(DroneStateMachine.State.CLOSE);
                        currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftFTC.State.INITSTRAFE;
                        waitTimer.reset();
                    }
                    break;

                case INITSTRAFE:
                    if(waitTimer.milliseconds() >= 1000){
                        drive.followTrajectorySequenceAsync(traj0);
                        currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftFTC.State.LIFTUP;
                        waitTimer.reset();
                    }
                    break;

                case LIFTUP:
                    if(!drive.isBusy() && waitTimer.milliseconds() >= 750){
                        drive.robot.getLiftSubsystem().extend(MID);
                        if(tf){
                            currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftFTC.State.TODROP;
                        }
                        else {
                            currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftFTC.State.FORWARD;
                            tf = true;
                        }
                        waitTimer.reset();
                    }
                    break;

                case FORWARD:
                    if (waitTimer.milliseconds() >= 2000) {
                        drive.followTrajectorySequenceAsync(traj1);
                        currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftFTC.State.PRELOAD;
                        waitTimer.reset();
                    }
                    break;

                case PRELOAD:
                    if(!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(traj2);
                        currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftFTC.State.MOVEARM;
                        waitTimer.reset();
                    }
                    break;

                case MOVEARM:
                    if(!drive.isBusy()){
                        drive.robot.getOuttakeSubsystem().getStateMachine().updateState(OuttakeStateMachine.State.PICKUP);
                        currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftFTC.State.CLAWOPEN;
                        waitTimer.reset();
                    }
                    break;

                case CLAWOPEN:
                    if(waitTimer.milliseconds() >= 1000){
                        drive.robot.getDroneSubsystem().getStateMachine().updateState(DroneStateMachine.State.OPEN);
                        currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftFTC.State.MOVEARMBACK;
                        waitTimer.reset();
                    }
                    break;

                case MOVEARMBACK:
                    if(waitTimer.milliseconds() >= 1000){
                        drive.robot.getOuttakeSubsystem().getStateMachine().updateState(OuttakeStateMachine.State.FORWARD);
                        currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftFTC.State.LIFTDOWN;
                        waitTimer.reset();
                    }
                    break;

                case LIFTDOWN:
                    if(waitTimer.milliseconds() >= 1000){
//                        if(counter < 1) {
//                            drive.robot.getLiftSubsystem().extend(6d);
//                            currentState = State.TOSTACK;
//                        }
//                        else{
                        drive.robot.getLiftSubsystem().retract();
                        currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftFTC.State.PARK;
                        // }
                        waitTimer.reset();
                    }
                    break;

                case TOSTACK:
                    if(waitTimer.milliseconds() >= 2000){
                        drive.followTrajectorySequenceAsync(traj3);
                        currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftFTC.State.GRAB;
                        waitTimer.reset();
                    }
                    break;

                case GRAB:
                    if(!drive.isBusy()){
                        drive.robot.getDroneSubsystem().getStateMachine().updateState(DroneStateMachine.State.CLOSE);
                        currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftFTC.State.LIFTUP;
                        waitTimer.reset();
                    }
                    break;

                case TODROP:
                    if(waitTimer.milliseconds() >= 2000){
                        drive.followTrajectorySequenceAsync(traj4);
                        if(counter < 1){
                            currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftFTC.State.MOVEARM;
                            waitTimer.reset();
                            counter++;
                        }
                        else {
                            currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftFTC.State.LIFTDOWN;
                            waitTimer.reset();
                        }
                    }
                    break;

                case PARK:
                    if(waitTimer.milliseconds() >= 2000){
                        if(placement == 1){
                            drive.followTrajectorySequenceAsync(location1);
                        }
                        else if(placement == 2){
                            drive.followTrajectorySequenceAsync(location2);
                        }
                        else if(placement == 3){
                            drive.followTrajectorySequenceAsync(location3);
                        }
                        currentState = org.firstinspires.ftc.teamcode.team.auto.BlueLeftFTC.State.IDLE;
                        waitTimer.reset();
                    }
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
}
