#include"fsm.h"

// user application
int main(){
    int i;
    cout<<"Press Control+C or a negative number to exit the program"<<"\n";
    finiteStateMachine* robot_fsm = new finiteStateMachine();
    while (true){
        cin >> i; 
        if (i >= 0){
            robot_event e = static_cast<robot_event>(i);
            robot_fsm->event(e);
        }
        else{
            break;
        }
    }
    delete robot_fsm;
    return 0;
}