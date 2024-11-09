package om.self.ezftc.core.part;

import om.self.beans.core.BeanManager;
import om.self.task.core.Group;
import om.self.task.event.EventManager;

public interface PartParent {
    String getName();

    Group getTaskManager();

    EventManager getEventManager();

    BeanManager getBeanManager();
}
