package frc.robot.utils;

import edu.wpi.first.networktables.*;

import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

public class GenericNTButton {
    private final String name;
    private boolean defaultValue;
    private final AtomicBoolean value = new AtomicBoolean(false);
    private final BooleanTopic entry;
    private NetworkTable subTable;
    private final BooleanSupplier controllerConnected;

    public GenericNTButton(String name, NetworkTable table, boolean defaultValue) {
        this.name = name;
        this.defaultValue = defaultValue;

        controllerConnected = () -> false;

        entry = table.getBooleanTopic(name);
        entry.getEntry(defaultValue).setDefault(defaultValue);

        NetworkTableInstance.getDefault().addListener(entry, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            value.set(event.valueData.value.getBoolean());
        });

        entry.publish();
    }

    public GenericNTButton(String name, NetworkTable table, BooleanSupplier controllerConnected, boolean defaultValue) {
        this.name = name;
        this.defaultValue = defaultValue;

        this.controllerConnected = controllerConnected;

        entry = table.getBooleanTopic(name);
        entry.getEntry(defaultValue).setDefault(defaultValue);

        NetworkTableInstance.getDefault().addListener(entry, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            if (!controllerConnected.getAsBoolean()) {
                value.set(event.valueData.value.getBoolean());
            }
        });

        entry.publish();
    }

    public GenericNTButton(String name, NetworkTable table, String group, BooleanSupplier controllerConnected, boolean defaultValue) {
        this.name = name;
        this.defaultValue = defaultValue;

        this.controllerConnected = controllerConnected;

        subTable = table.getSubTable(group);
        entry = subTable.getBooleanTopic(name);
        entry.getEntry(defaultValue).setDefault(defaultValue);

        NetworkTableInstance.getDefault().addListener(entry, EnumSet.of(NetworkTableEvent.Kind.kValueLocal), event -> {
            value.set(event.valueData.value.getBoolean());
        });

        NetworkTableInstance.getDefault().addListener(entry, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            if (!controllerConnected.getAsBoolean()) {
                value.set(event.valueData.value.getBoolean());
                subTable.getTopics().forEach(topic -> {
                    GenericEntry subTableEntry = topic.getGenericEntry();
                    if (!topic.getName().equals(name)) subTableEntry.setBoolean(false);
                });
                entry.getEntry(defaultValue).set(true);
            }
        });

        entry.publish();
    }

    public boolean get() {
        return value.get();
    }

    public BooleanEntry getEntry() {
        return entry.getEntry(defaultValue);
    }

    public void set(boolean value) {
        entry.getEntry(defaultValue).set(value);
    }

    public void setDefaultValue(boolean value) {
        defaultValue = (boolean) value;
    }

}
