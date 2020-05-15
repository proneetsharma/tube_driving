#include "fsm.h"
#include <assert.h>

typedef struct {
    robot_state current;
    robot_event event;
    robot_state next;
} state_transition_element;

typedef struct {
    robot_state state;
    void (finiteStateMachine::*func)(robot_state);
}state_function;

// assign proper names to event
state_transition_element state_transition_table[] = {
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

/* we will modify this when all the state functions are implemented
   right now we are just calling function that prints the transition state
*/
static state_function transition_map[] = {
    {idle , &finiteStateMachine::stateFunction},
    {cruising , &finiteStateMachine::stateFunction},
    {entry_corner , &finiteStateMachine::stateFunction},
    {align_accel , &finiteStateMachine::stateFunction},
    {align_decel , &finiteStateMachine::stateFunction},
    {turn , &finiteStateMachine::stateFunction},
    {entry_straight , &finiteStateMachine::stateFunction},
    {inter_straight , &finiteStateMachine::stateFunction},
    {overtake_spacious , &finiteStateMachine::stateFunction},
    {overtake_tight , &finiteStateMachine::stateFunction},
};

// remove this array once the state functions are implemented
string s[10] = { "idle", "cruising", "entry_corner", "align_accel", "align_decel", \
                "turn","entry_straight","inter_straight","overtake_spacious","overtake_tight"};

finiteStateMachine::finiteStateMachine(){
    maximum_states = maxi_states;
    current_state = idle;
    // generate an internal event and make transition from idle to cruising.
    event(event_0);  
}

// remove this function once all the state functions are implemented
void finiteStateMachine::stateFunction(robot_state rs){
    cout<<"current state: "<< s[rs] <<"\n";
}

// add one more paramter for event data
void finiteStateMachine::event(robot_event new_event){
    cout <<"previous state: " << s[state_transition_table[new_event].current] << "\n";
    if(current_state == state_transition_table[new_event].current){
        event_generated = true;
        current_state = state_transition_table[new_event].next;
        finiteStateEngine();
    }
    else{
        // delete the event data if any
        cout<<"Encountered illegal transition"<<"\n";
    }
}

void finiteStateMachine::finiteStateEngine(){
    while (event_generated){
        event_generated = false;
        assert(current_state < maxi_states);        
        (this->*transition_map[current_state].func)(current_state);
        //delete event data once used
    }
}
