#include"fsm.h"

// user application
int main(){
    int i;
    std::cout<<"Press Ctrl + C or a negative number to exit the program"<<"\n";
    finiteStateMachine* robot_fsm = new finiteStateMachine();
    while (true){
        std::cin >> i; 
        if (i >= 0){
            RobotEvent e = static_cast<RobotEvent>(i);
            robot_fsm->event(e);
        }
        else{
            break;
        }
    }
    delete robot_fsm;
    return 0;
}