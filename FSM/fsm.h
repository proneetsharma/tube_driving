#ifndef FSM_H
#define FSM_H

#include <iostream>
#include <bits/stdc++.h> 
using namespace std;

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
} robot_state;


// define all external and internal events
// assign proper names to events
typedef enum{
    event_0,event_1,event_2,event_3,event_4,event_5,event_6,event_7,event_8, \
    event_9,event_10,event_11,event_12,event_13,event_14,event_15,event_16,event_17
}robot_event;

class finiteStateMachine
{
    private:
        bool event_generated;
        robot_state current_state;
        unsigned char maximum_states;
        void finiteStateEngine();
    public:
        finiteStateMachine();
        ~finiteStateMachine(){cout<<"Deleted FSM"<<"\n";};
        void event(robot_event new_state);
        // implement all state functions as public methods
        void stateFunction(robot_state rs);
};

#endif