//ici c'est cam.c
#include "cam.cpp"
int main(){
    int code= system(cam.cpp);

    // checking if the command was executed successfully
    if (code == 0) {
        cout << "Command executed successfully, c'est good pouponnn" << endl;
    }
    else {
        cout << "Command execution failed or returned "
                "non-zero: "
             << returnCode << endl;
    }
 
    return 0;
}
