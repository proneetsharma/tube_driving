#include "fsm.h"
#include <assert.h>

typedef struct {
    RobotState state;
    void (finiteStateMachine::*func)(RobotState);
}StateFunction;

/* we will modify this when all the state functions are implemented
   right now we are just calling function that prints the transition state
*/
static StateFunction transition_map[] = {
    {idle , &finiteStateMachine::currentState},
    {cruising , &finiteStateMachine::currentState},
    {entry_corner , &finiteStateMachine::currentState},
    {align_accel , &finiteStateMachine::currentState},
    {align_decel , &finiteStateMachine::currentState},
    {turn , &finiteStateMachine::currentState},
    {entry_straight , &finiteStateMachine::currentState},
    {inter_straight , &finiteStateMachine::currentState},
    {overtake_spacious , &finiteStateMachine::currentState},
    {overtake_tight , &finiteStateMachine::currentState}
};

finiteStateMachine::finiteStateMachine(){
    maximum_states = maxi_states;
    current_state = idle;
    // generate an internal event and make transition from idle to cruising.
    event(event_0);  
}

// remove this function once all the state functions are implemented
void finiteStateMachine::currentState(RobotState rs){
    std::cout<<"current state: "<< s[rs] <<"\n";
}

// add one more paramter for event data
void finiteStateMachine::event(RobotEvent new_event){
    std::cout <<"previous state: " << s[state_transition_table[new_event].current] << "\n";
    if(current_state == state_transition_table[new_event].current){
        event_generated = true;
        current_state = state_transition_table[new_event].next;
        finiteStateEngine();
    }
    else{
        // delete the event data if any
        std::cout<<"Encountered illegal transition"<<"\n";
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
