package om.self.ezftc.core.part;

import om.self.task.core.Group;

public abstract class StatefullPart<PARENT extends PartParent, SETTINGS, HARDWARE, STATE> extends Part<PARENT, SETTINGS, HARDWARE>{
    private boolean lockState = false;
    private STATE state;
    public boolean allowNull = false;
    private STATE tempState;

    public StatefullPart(PARENT parent, String name) {
        super(parent, name);
    }

    public StatefullPart(PARENT parent, String name, Group taskManager) {
        super(parent, name, taskManager);
    }

    public boolean isStateLocked() {
        return lockState;
    }

    /**
     * checks if there is currently a temp state active
     * @return the there is a temp state active
     */
    public boolean isStateTemp(){
        return tempState != null;
    }

    public void setLockState(boolean lockState) {
        this.lockState = lockState;
    }

    public void setState(STATE state) {
        if(lockState) return;
        if (state == null && !allowNull) throw new RuntimeException("allowNull has been set to false so states can not be null!");

        state = sanitizeState(state);
        if(!isStateTemp()) onStateUpdate(state);
        this.state = state;
    }

    public void activateTempState(STATE state){
        if (state == null && !allowNull) throw new RuntimeException("allowNull has been set to false so states can not be null!");

        state = sanitizeState(state);
        onStateUpdate(state);
        tempState = state;
    }

    public void killTempState(){
        tempState = null;
        onStateUpdate(state);
    }

    public STATE getTargetState(){
        if(isStateTemp()) return tempState;
        return state;
    }

    public abstract STATE getCurrentState();

    public abstract void onStateUpdate(STATE state);

    public abstract STATE sanitizeState(STATE state);
}
