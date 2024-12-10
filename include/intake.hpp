#include "include.hpp"
#include <cstdint>
#include <optional>
extern bool JAM_PROTECTION_ACTIVE;
extern bool JAM_PROTECTION_INACTIVE;

extern pros::Mutex ringControlMutex;

class IntakeControl{
    private:
    const uint16_t jamCycleThreshold = 8;
    const uint16_t dejamThreshold = 20;

    const uint16_t detectThreshold = 3;
    const uint16_t redDetectThreshold = 1;
    const uint16_t blueDetectThreshold = 2;

    const uint16_t NoDetectThreshold = 5;

    int16_t intakeVel;
    int16_t jamThresh;
    int32_t jamSpeed;
   
    uint8_t  intakeFlag = 0;
    uint16_t jamCycles;
    uint16_t reverseCycles;
    uint16_t deadCycles;
    uint16_t detectCycles;
    uint16_t noDetectCycles;
    
    bool jamProtection;
    bool isRunning = false;
    bool notifyOff = false;
    bool lastJamDead = true;
    bool lookingRed = false;
    bool lookingBlue = false; 
   
    public:
    IntakeControl(){
        setJamThresh(200);
        setJamSpeed(-400);
        setJamProtection(JAM_PROTECTION_ACTIVE);
    }

    ~IntakeControl(){
        intake.move_voltage(0);
    }

    void startIntake();
    void stopIntake();
    void setIntake(int16_t velocity, std::optional<autoColor> color);
    void setJamThresh(int16_t velocity);
    void setJamSpeed(int16_t velocity);
    void setJamProtection(bool state);

    void run();

    int32_t intakeSpeed;

}; 
extern IntakeControl conveyor; //declration for intakecontrol obj 

//Set the Intakes speed to run at when operating
void setIntake(int16_t velocity, std::optional<autoColor> color);
//Begin running the Intake at the previously specified speed
void startIntake();
//Stop running the Intake (Does not continously run at zero, rather set speed to zero then quits)
void stopIntake();

//float AIOtimeOut;
void IntakeControlSystem_fn(void *param);