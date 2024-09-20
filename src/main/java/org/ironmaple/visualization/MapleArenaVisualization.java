package org.ironmaple.visualization;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class MapleArenaVisualization {
    public static final class ObjectInArena {
        private final String typeName;
        private final Pose3d pose3d;

        public ObjectInArena(String typeName, Pose3d pose3d) {
            this.typeName = typeName;
            this.pose3d = pose3d;
        }
    }

    private static Map<String, Set<Pose3d>> organizeObjectPosesByType(ObjectInArena... objects) {
        final Map<String, Set<Pose3d>> objectsOfCertainTypePoses = new HashMap<>();

        for (ObjectInArena object:objects) {
            if (!objectsOfCertainTypePoses.containsKey(object.typeName))
                objectsOfCertainTypePoses.put(object.typeName, new HashSet<>());
            objectsOfCertainTypePoses.get(object.typeName).add(object.pose3d);
        }

        return objectsOfCertainTypePoses;
    }

    /**
     * @param path the path where the visualization should be logged to, such as /FieldVisualization
     * */
    private static void visualizeWithAdvantageKit(String path, Map<String, Set<Pose3d>> objectsOfCertainTypePoses) {
        for (String type: objectsOfCertainTypePoses.keySet())
            Logger.recordOutput(path + "/Objects/" + type, objectsOfCertainTypePoses.get(type).stream().toArray(Pose3d[]::new));
    }
}
