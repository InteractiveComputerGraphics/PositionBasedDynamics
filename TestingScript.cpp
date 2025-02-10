#include<stdlib.h>
#include<iostream>
#include<string>

using namespace std;

int main()
{
    cout << "Write filename addition: ";
    string input;
    cin >> input;
    string defaultCommand =  ".\\FluidParticleSpatialTest.exe";
    string command = defaultCommand + " d" + " DefaultNum"+input;
    cout << "Currently doing: default" << endl;
    system(command.c_str());

    for(int i = -2; i < 3; i++)
    {
        string num = to_string(i);
        cout << "Currently doing: " << num << endl;
        command = defaultCommand + " " + num + " " + num+ "Num"+input;
        system(command.c_str());
    }

    return 0;
}