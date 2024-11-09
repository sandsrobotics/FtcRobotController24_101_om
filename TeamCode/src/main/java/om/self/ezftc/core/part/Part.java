package om.self.ezftc.core.part;

import om.self.beans.core.BeanManager;
import om.self.ezftc.core.Robot;
import om.self.task.core.Group;
import om.self.task.event.EventManager;

/**
 * Provides a simple configuration of events, beans, and tasks that allow you to make extensions(ex: robot parts or telemetry).
 * PLEASE make new documentation when creating a part.
 * <br>
 * <br>
 * USED EVENTS:
 * <ul>
 *     <li>START_(part name) --> START_(part name)_(extension name)</li>
 *     <li>STOP_(part name) --> STOP_(part name)_(extension name)</li>
 * </ul>
 * @param <PARENT> the type of the part you want to extend
 */
public abstract class Part<PARENT extends PartParent, SETTINGS, HARDWARE> implements PartParent {

    //basic
    public final PARENT parent;
    public final Group taskManager;
    public final EventManager eventManager;
    public final String name;

    private SETTINGS settings;
    private HARDWARE hardware;

    private boolean running;

    /**
     * A constructor for Part that creates task and event managers attached to the parents task and event managers
     * @param parent the parent of this part(CAN NOT BE NULL)
     * @param name the name of this part (used to register an event manager and task manager so it must be unique in the scope of the parent)
     */
    public Part(PARENT parent, String name) {
        this.parent = parent;
        this.name = name;
        taskManager = new Group(name,  parent.getTaskManager());
        eventManager = new EventManager(name, parent.getEventManager());
        construct();
    }

    /**
     * A constructor for Parts that create an event managers attached to the parents and a task managers attached to the custom task manager
     * @param parent the parent of this part(CAN NOT BE NULL)
     * @param name the name of this part (used to register an event manager and task manager so it must be unique in the scope of the parent)
     * @param taskManager the custom task manager that is the parent for this parts task manager
     */
    public Part(PARENT parent, String name, Group taskManager){
        this.parent = parent;
        this.name = name;
        this.taskManager = new Group(name, taskManager);
        eventManager = new EventManager(name, parent.getEventManager());
        construct();
    }

    private void construct(){
        //-----event manager-----//
        //make/attach start
        eventManager.attachToEvent(Robot.Events.INIT, "onInit", this::onInit);
        //make/attach start
        eventManager.attachToEvent(Robot.Events.START, "onStart", () -> {
            running = true;
            onStart();
        });
        //make/attach stop
        eventManager.attachToEvent(Robot.Events.STOP, "onStop", () -> {
            running = false;
            onStop();
        });
        eventManager.attachToEvent(Robot.Events.STOP, "stop taskManager", () -> taskManager.runCommand(Group.Command.PAUSE));
        //add bean!!
        getBeanManager().addBean(this, this::onBeanLoad, true, false);
    }


    @Override
    public String getName() {
        return name;
    }

    @Override
    public Group getTaskManager(){
        return taskManager;
    }

    @Override
    public EventManager getEventManager() {
        return eventManager;
    }

    public void triggerEvent(String event){
        eventManager.triggerEventRecursively(event);
    }

    @Deprecated
    public void triggerEvent(Enum event){
        eventManager.triggerEventRecursively(event);
    }

    @Override
    public BeanManager getBeanManager(){
        return parent.getBeanManager();
    }

    public SETTINGS getSettings() {
        return settings;
    }

    public void setSettings(SETTINGS settings) {
        onSettingsUpdate(settings);
        this.settings = settings;
    }

    public HARDWARE getHardware() {
        return hardware;
    }

    public void setHardware(HARDWARE hardware) {
        onHardwareUpdate(hardware);
        this.hardware = hardware;
    }

    public void setConfig(SETTINGS settings, HARDWARE hardware){
        setSettings(settings);
        setHardware(hardware);
    }

    public boolean isRunning() {
        return running;
    }

    public abstract void onBeanLoad();

    /**
     * WARNING: beans may not be loaded onInit, please use onStart for beans
     */
    public abstract void onInit();

    public void onSettingsUpdate(SETTINGS settings){}

    public void onHardwareUpdate(HARDWARE hardware){}

    public abstract void onStart();

    public abstract void onStop();
}
