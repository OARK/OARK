 package org.luminousmonkey.dynamiclayout.config;

public class WheelMotor extends Motor {
    private String name;
    private String displayText;
    private int id;
    private int initPosition;
    private int minPosition, maxPosition;

    public WheelMotor() {
    }

    public void setDisplayText(String displayName) {
        this.displayText = displayName;
    }

    public String getDisplayText() {
        return this.displayText;
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getName() {
        return this.name;
    }

    public void setId(int id) {
        this.id = id;
    }

    public int getId() {
        return this.id;
    }

    public void setInitPosition(int initPosition) {
        this.initPosition = initPosition;
    }

    public int getInitPosition() {
        return this.initPosition;
    }

    public void setMinPosition(int minPosition) {
        this.minPosition = minPosition;
    }

    public int getMinPosition() {
        return this.minPosition;
    }

    public void setMaxPosition(int maxPosition) {
        this.maxPosition = maxPosition;
    }

    public int getMaxPosition() {
        return this.maxPosition;
    }

    public boolean usesAbsolutePosition() {
        return false;
    }

    @Override
    public String toString() {
        return "Name: " + this.getName() + ", " +
            "ID: " + this.getId() + ", " +
            "InitPos: " + this.getInitPosition() + ", " +
            "MinPos: " + this.getMinPosition() + ", " +
            "MaxPos: " + this.getMaxPosition() + ".";
    }
}
