package org.firstinspires.ftc.teamcode.team.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.team.subsystems.ITDLiftSubsystem;

import java.util.function.DoubleConsumer;

//The following commented lines are from ElevatorStateMachine before deleting
//For lift stop calculations the POLE Heights from the Floor
//  High = 33.75 inches,
//  Mid = 23.75 inches,
//  Low = 13.75 inches

public class ITDLiftStateMachine implements IState<ITDLiftStateMachine.State> {
    private static DoubleConsumer runExtension;
    private ITDLiftSubsystem itdLiftSubsystem;
    private State state;
    private State desiredState;

    public ITDLiftStateMachine(ITDLiftSubsystem itdliftSubsystem) {
        setItdLiftSubsystem(itdliftSubsystem);
        setState(State.IDLE);
        setDesiredState(State.IDLE);
    }

    @Override
    public void updateState(State state) {
        setDesiredState(state);
    }

    @Override
    public boolean hasReachedStateGoal() {
        return getItdLiftSubsystem().closeToSetpoint(1 / 4d) && ITDLiftSubsystem.getExtensionProfile().isDone();
    }

    @Override
    public boolean hasReachedStateGoal(State state) {
        return state.equals(getState()) && hasReachedStateGoal();
    }

    @Override
    public boolean attemptingStateChange() {
        return !getState().equals(getDesiredState());
    }

    @Override
    public State getState() {
        return state;
    }

    @Override
    public State getDesiredState() {
        return desiredState;
    }

    @Override
    public String getName() {
        return "ITDLift State Machine";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) { //new change
        if (attemptingStateChange()) {
            setState(getDesiredState());
            if (getRunExtension() != null) {
                getRunExtension().accept(ITDLiftSubsystem.getDesiredSetpoint());
            }
        }
    }

    private void setState(State state) {this.state = state;}

    private void setDesiredState(State desiredState) {this.desiredState = desiredState;}

    public enum State implements Namable {
        IDLE("Idle"), EXTEND("Extend"), RETRACT("Retract");

        private final String name;

        State(final String name) {
            this.name = name;
        }

        @Override
        public String getName() {
            return name;
        }
    }

    public ITDLiftSubsystem getItdLiftSubsystem() {
        return itdLiftSubsystem;
    }

    public void setItdLiftSubsystem(ITDLiftSubsystem itdLiftSubsystem) {this.itdLiftSubsystem = itdLiftSubsystem;}

    public static DoubleConsumer getRunExtension() {return runExtension;}

    public static void setRunExtension(DoubleConsumer runExtension) {
        ITDLiftStateMachine.runExtension = runExtension;
    }
}
