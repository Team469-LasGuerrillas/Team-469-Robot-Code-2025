package frc.robot.util;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.ParentDevice;

import frc.robot.subsystems.constants.ElevatorConstants;

public class Music {
    static Orchestra music = new Orchestra();

    public static void loadSong(String song) {
        music.loadMusic(song);
    }

    public static void play() {
        music.play();
    }

    public static void pause() {
        music.pause();
    }

    public static void stop() {
        music.stop();
    }

    public static void addInstrument(ParentDevice instrument) {
        music.addInstrument(instrument, 0);
    }
}
