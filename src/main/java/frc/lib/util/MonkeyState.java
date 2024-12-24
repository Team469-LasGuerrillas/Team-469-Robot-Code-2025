package frc.lib.util;

public interface MonkeyState{

  public default void addState(double timestamp) {}

  public default MonkeyState getState(double timestamp) {
    return null;
  }
}
