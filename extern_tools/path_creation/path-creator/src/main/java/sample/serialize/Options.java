package sample.serialize;


public enum Options {
    lineTo(true, false, InputType.POSITION),
    splineTo(true, false, InputType.POSITION, InputType.NUMBER),
    turnTo(true, false, InputType.NUMBER),
    strafeTo(true, false, InputType.POSITION),
    reverse(false, false);

    // Movement actions are treated specially (create new point on field)
    public boolean movementAction;
    public boolean nonDriveAction;

    public InputType[] types;


    Options(boolean movementAction, boolean nonDriveAction, InputType... types) {
        this.movementAction = movementAction;
        this.types = types;
    }
}