package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team.auto.ITDBaseLACH;
import org.firstinspires.ftc.teamcode.team.states.ITDLiftStateMachine;
import org.firstinspires.ftc.teamcode.team.states.ITDArmStateMachine;
import org.firstinspires.ftc.teamcode.team.states.ITDClawStateMachine;
import org.firstinspires.ftc.teamcode.team.states.ITDClawArmStateMachine;


/*
 * This {@code class} acts as the driver-controlled program for FTC team 16598 for the Into The Deep
 * challenge. By extending {@code ITDTeleopRobotCHALLC}, we already have access to all the robot subsystems,
 * so only tele-operated controls need to be defined here.
 *
 * The controls for this robot are:
 *  User 1:
 *      Drive:
 *          Left & Right joysticks      -> Mecanum drive
 *          Left-Bumper                 -> Decrease robot speed .7x
 *          Right-Bumper                -> Normal robot speed 1x
 *      Lift:
 *          Y-Button                    -> Extend lift to "Out" position
 *          B-Button                    -> Extend lift to "In" position
 *          A-Button                    -> Retract lift to starting position
 *      Arm:
 *          Left_trigger                -> Turn arm to "PickUp" position
 *          Right_trigger               -> Turn arm to "Drop" position
 *          X-Button (pressed)          -> Turn arm to Starting position
 *
 *  User 2:
 *      Claw:
 *          Left-trigger                ->
 *          Right-trigger               ->
 *          A-button (pressed)          ->
 *          Y-button (pressed)          ->
 *      ClawArm:
 *          Dpad-up                     ->
 *          Dpad-down                   ->
 *
 */
@TeleOp(name = "ITD TeleOp CHALLC", group = "Main")
public class ITDTeleopCHALLC extends ITDTeleopRobotCHALLC {

    private double currentTime = 0; // keep track of current time
    private double speedMultiplier = 0.7;
    //these are based on LiftTest
    private static final double Out = 14d;
    private static final double In = 8d;
    private static final double High = 17d;
    private static final double PickUp = 3d;
    private static final double Drop = 4.7d;





    private Pose2d poseEstimate;

    @Override
    public void init(){
        drive = new ITDBaseLACH(hardwareMap, true);
        super.init();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        super.loop();
        drive.update();
        poseEstimate = drive.getPoseEstimate();
//---------------------------------------------------------------------------------------------------------------------------------------------------------
        //Gamepad 1

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * speedMultiplier,
                        -gamepad1.left_stick_x * speedMultiplier,
                        -gamepad1.right_stick_x * speedMultiplier
                )
        );

        //This changes the speed the robot moves at
        if (getEnhancedGamepad1().isLeftBumperJustPressed()) {
            speedMultiplier = 0.7;
        }
        if (getEnhancedGamepad1().isRightBumperJustPressed()) {
            speedMultiplier = 1.0;
        }

        //Arm
        if (getEnhancedGamepad1().getLeft_trigger() > 0) {
            drive.robot.getITDArmSubsystem().setSetpoint(PickUp);
        }
        if (getEnhancedGamepad1().getRight_trigger() > 0) {
            drive.robot.getITDArmSubsystem().setSetpoint(Drop);
        }
        if (getEnhancedGamepad1().isxJustPressed()) {
            drive.robot.getITDArmSubsystem().retract();
        }




//---------------------------------------------------------------------------------------------------------------------------------------------------------
        //Gamepad 2


        //ClawArm
        if (getEnhancedGamepad2().getLeft_trigger() > 0) {
            drive.robot.getITDClawArmSubsystem().getStateMachine().updateState(ITDClawArmStateMachine.State.PICKUP);
        }
        if (getEnhancedGamepad2().getRight_trigger() > 0) {
            drive.robot.getITDClawArmSubsystem().getStateMachine().updateState(ITDClawArmStateMachine.State.DROP);
        }

        //Claw
        if (getEnhancedGamepad2().getLeft_trigger() > 0) {
            drive.robot.getITDClawSubsystem().getStateMachine().updateState(ITDClawStateMachine.State.OPEN);
        }
        if (getEnhancedGamepad2().getRight_trigger() > 0) {
            drive.robot.getITDClawSubsystem().getStateMachine().updateState(ITDClawStateMachine.State.CLOSE);
        }

        //Lift
        if(getEnhancedGamepad2().isDpadUpJustPressed()){
            drive.robot.getITDLiftSubsystem().extend(High);
        }
        if(getEnhancedGamepad2().isDpadRightJustPressed()){
            drive.robot.getITDLiftSubsystem().extend(Out);
        }
        if(getEnhancedGamepad2().isDpadLeftJustPressed()){
            drive.robot.getITDLiftSubsystem().extend(In);
        }
        if(getEnhancedGamepad2().isaJustPressed()){
            drive.robot.getITDLiftSubsystem().retract();
        }

        telemetry.addData("Lift State: ", drive.robot.getITDLiftSubsystem().getStateMachine().getState());
        telemetry.addData("Arm State: ", drive.robot.getITDArmSubsystem().getStateMachine().getState());
        telemetry.addData("Claw: ", drive.robot.getITDClawSubsystem().getStateMachine().getState());
        telemetry.addData("ClawArm: ", drive.robot.getITDClawArmSubsystem().getStateMachine().getState());


        updateTelemetry(telemetry);
        currentTime = getRuntime();
    }

}