package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team.auto.ITDBaseLACH;
import org.firstinspires.ftc.teamcode.team.states.ITDClawArmStateMachine;
import org.firstinspires.ftc.teamcode.team.states.ITDClawStateMachine;


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
@TeleOp(name = "ITD TeleOp CC", group = "Main")
public class ITDTeleopCC_Test extends ITDTeleopRobotCHALLC {

    private double currentTime = 0; // keep track of current time
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
        //Gamepad 2


        //ClawArm
        if (getEnhancedGamepad2().getLeft_trigger() > 0) {
            drive.robot.getITDClawArmSubsystem().getStateMachine().updateState(ITDClawArmStateMachine.State.PICKUP);
        }
        if (getEnhancedGamepad2().getRight_trigger() > 0) {
            drive.robot.getITDClawArmSubsystem().getStateMachine().updateState(ITDClawArmStateMachine.State.DROP);
        }
        if (getEnhancedGamepad1().isxJustPressed()) {
            drive.robot.getITDClawArmSubsystem().getStateMachine().updateState(ITDClawArmStateMachine.State.IDLE);
        }

        //Claw
        if (getEnhancedGamepad2().isRightBumperJustPressed()) {
            drive.robot.getITDClawSubsystem().getStateMachine().updateState(ITDClawStateMachine.State.OPEN);
        }
        if (getEnhancedGamepad2().isLeftBumperJustPressed()) {
            drive.robot.getITDClawSubsystem().getStateMachine().updateState(ITDClawStateMachine.State.CLOSE);
        }


        telemetry.addData("Claw: ", drive.robot.getITDClawSubsystem().getStateMachine().getState());
        telemetry.addData("ClawArm: ", drive.robot.getITDClawArmSubsystem().getStateMachine().getState());


        updateTelemetry(telemetry);
        currentTime = getRuntime();
    }

}
