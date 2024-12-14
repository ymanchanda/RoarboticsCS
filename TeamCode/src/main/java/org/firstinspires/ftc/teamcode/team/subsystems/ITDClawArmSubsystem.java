package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.team.states.ITDClawArmStateMachine;

public class ITDClawArmSubsystem implements ISubsystem<ITDClawArmStateMachine, ITDClawArmStateMachine.State> {
    private static ITDClawArmStateMachine itdClawArmStateMachine;
    private RevServo ClawArmServo;


    public ITDClawArmSubsystem(RevServo clawServo){
        setClawStateMachine(new ITDClawArmStateMachine());
        setGripperServo(clawServo);
    }

    @Override
    public ITDClawArmStateMachine getStateMachine() {
        return itdClawArmStateMachine;
    }

    @Override
    public ITDClawArmStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }

    @Override
    public String getName() {
        return "Claw Arm Subsystem";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getClawServo().setPosition(getState().getPosition());
    }

    public static void setClawStateMachine(ITDClawArmStateMachine gripperStateMachine) {
        ITDClawArmSubsystem.itdClawArmStateMachine = gripperStateMachine;
    }

    public RevServo getClawServo() {
        return ClawArmServo;
    }


    public void setGripperServo(RevServo clawServo) {
        this.ClawArmServo = clawServo;
    }
}