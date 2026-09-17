// RC_Controlls.h
#ifndef RC_CONTROLS_H
#define RC_CONTROLS_H

struct ControllerInput {
    //Sticks:
    float throttle;
    float yaw;
    float pitch;
    float roll;
    //Switches:
    int SA;
    int SB;
    int SC;
    int SD;
    int SE;
    int SF;
    //Tuning pots:
    float S1;
    float S2;
};

int getSwitchPosition(int channelValue);
ControllerInput listen_channels(ControllerInput input, int THR, int YAW, int PITCH, int ROLL, int SA, int SB, int SC, int SD, int SE, int SF, int S1, int S2);

#endif // RC_CONTROLS_H