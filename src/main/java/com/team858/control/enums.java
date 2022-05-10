package com.team858.control;

public class enums {
    public enum Direction {
        NONE(0), UP(1), RIGHT(2), DOWN(3), LEFT(4);
    
        private final int value;
        private Direction(int value) {
            this.value = value;
        }
    
        public int getValue() {
            return value;
        }
    }
}
