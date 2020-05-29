#ifndef FSM_H
#define FSM_H

#include <iostream>
// define all states of the finite state machine
typedef enum{
    idle,
    cruising,
    entry_corner,
    align_accel,
    align_decel,
    turn,
    entry_straight,
    inter_straight,
    overtake_spacious,
    overtake_tight,
    maxi_states,
} RobotState;

// define all external and internal events
// assign proper names to events
typedef enum{
    event_0,event_1,event_2,event_3,event_4,event_5,event_6,event_7,event_8, \
    event_9,event_10,event_11,event_12,event_13,event_14,event_15,event_16,event_17
}RobotEvent;

typedef struct {
    RobotState current;
    RobotEvent event;
    RobotState next;
} StateTransitionElement;

// assign proper names to event
static StateTransitionElement state_transition_table[] = {
    {idle, event_0 , cruising},
    {cruising, event_1 ,entry_corner},
    {entry_corner, event_2, align_accel},
    {align_accel, event_3 , align_decel},
    {align_decel, event_4 , turn},
    {align_accel, event_5 , turn},
    {turn, event_6 , cruising},
    {cruising, event_7 , entry_straight},
    {entry_straight, event_8 , inter_straight},
    {inter_straight, event_9 , cruising},
    {cruising, event_10 , overtake_spacious},
    {overtake_spacious, event_11 , cruising},
    {cruising, event_12 , overtake_tight},
    {overtake_tight, event_13 , cruising},
    {inter_straight, event_14 , overtake_spacious},
    {turn, event_15 , overtake_tight},
    {inter_straight, event_16 ,overtake_tight},
    {turn, event_17 , overtake_spacious}
};

// remove this array once the state functions are implemented
static std::string s[10] = { "idle", "cruising", "entry_corner", "align_accel", "align_decel", \
                "turn","entry_straight","inter_straight","overtake_spacious","overtake_tight"};

class finiteStateMachine
{
    private:
        bool event_generated;
        RobotState current_state;
        unsigned char maximum_states;
        void finiteStateEngine();
    public:
        finiteStateMachine();
        ~finiteStateMachine(){std::cout<<"Deleted FSM"<<"\n";};
        void event(RobotEvent new_event);
        // implement all state functions as public methods
        void currentState(RobotState rs);
};
#endif