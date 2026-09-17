// RC_Controlls.cpp
#include <AlfredoCRSF.h>
#include "RC_Controlls.h"

extern AlfredoCRSF crsf;

int getSwitchPosition(int channelValue) {
  if (channelValue < 1250) {
    return 0; // Läge ner
  } 
  else if (channelValue < 1750) {
    return 1; // Läge mitten
  } 
  else {
    return 2; // Läge upp
  }
}

ControllerInput listen_channels(ControllerInput input, int THR, int YAW, int PITCH, int ROLL, int SA, int SB, int SC, int SD, int SE, int SF, int S1, int S2) {
    crsf.update();

    if (crsf.isLinkUp()) {
        input.throttle = ((float)crsf.getChannel(THR) / 1000.0f) - 1; // Normalize to [0, 1]
        input.yaw = ((float)crsf.getChannel(YAW) / 500.0f) - 3.0f; // Normalize to [-1, 1]
        input.pitch = ((float)crsf.getChannel(PITCH) / 500.0f) - 3.0f; // Normalize to [-1, 1]
        input.roll = ((float)crsf.getChannel(ROLL) / 500.0f) - 3.0f; // Normalize to [-1, 1]

        // Sticks - (0, 1, 2) if possible
        input.SA = getSwitchPosition(crsf.getChannel(SA));
        input.SB = getSwitchPosition(crsf.getChannel(SB));
        input.SC = getSwitchPosition(crsf.getChannel(SC));
        input.SD = getSwitchPosition(crsf.getChannel(SD));
        input.SE = getSwitchPosition(crsf.getChannel(SE));
        input.SF = getSwitchPosition(crsf.getChannel(SF));

        input.S1 = (((float)crsf.getChannel(S1) / 500.0f) - 2); // Normalize to [0, 2]
        input.S2 = (((float)crsf.getChannel(S2) / 500.0f) - 2); // Normalize to [0, 2]
    }
    else {
        Serial.println("No radiolink");
        input.throttle = 0.0f;
        input.yaw = 0.0f;
        input.pitch = 0.0f;
        input.roll = 0.0f;
        input.SA = -1; // Indicate no link
        input.SB = -1; // Indicate no link
        input.SC = -1; // Indicate no link
        input.SD = -1; // Indicate no link
        input.SE = -1; // Indicate no link
        input.SF = -1; // Indicate no link
        input.S1 = -1; // Indicate no link
        input.S2 = -1; // Indicate no link
    }

    //Debugging output to serial monitor
    /*Serial.print("Josticks: \n");
    Serial.print("Throttle: ");
    Serial.print(input.throttle);
    Serial.print(" | Yaw: ");
    Serial.print(input.yaw);
    Serial.print(" | Pitch: ");
    Serial.print(input.pitch);
    Serial.print(" | Roll: ");
    Serial.print(input.roll);
    Serial.print("\nSwitches: \n");
    Serial.print("SA: ");
    Serial.print(input.SA);
    Serial.print(" | SB: ");
    Serial.print(input.SB);
    Serial.print(" | SC: ");
    Serial.print(input.SC);
    Serial.print(" | SD: ");
    Serial.print(input.SD);
    Serial.print(" | SE: ");
    Serial.print(input.SE);
    Serial.print(" | SF: ");
    Serial.print(input.SF);
    Serial.print(" | S1: ");
    Serial.print(input.S1);
    Serial.print(" | S2: ");
    Serial.println(input.S2);*/

    return input;
}