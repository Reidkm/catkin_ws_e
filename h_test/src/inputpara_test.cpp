
#include <iostream>
#include <string>
#include <string.h>
using namespace std;


void ShowUsage()

{

    cout << "Usage   : rec_user_arg <--name=your name> [Option]" << endl;

    cout << "Options :" << endl;

    cout << " --name=your name                  Your name, this option MUST be given." << endl;

    cout << " --occupation=your occupation      Your occupation, such as paladin." << endl;

    cout << " --camp=your camp                  Your camp, such as alliance." << endl;

    cout << " --help                            Print this help." << endl;


    return;
}

 

int main(int argc, char *argv[])

{

    // 如果用户没有输入参数，则提示错误信息并退出

    if (argc < 2)

    {
        cout << "No arguments, you MUST give an argument at least!" << endl;

        ShowUsage();

        return -1;
    }

 
    int nOptionIndex = 1;

    string strName;

    string strOccupation;

    string strCamp;

    while (nOptionIndex < argc)

    {

        // 获取用户姓名

        if (strncmp(argv[nOptionIndex], "--name=", 7) == 0)

        {

            strName = &argv[nOptionIndex][7];
            //strName = argv[nOptionIndex][7];

        }

        // 获取用户职业

        else if (strncmp(argv[nOptionIndex], "--occupation=", 13) == 0)

        {

            strOccupation = &argv[nOptionIndex][13];

        }

        // 获取用户阵营

        else if (strncmp(argv[nOptionIndex], "--camp=", 7) == 0)

        {

            strCamp = &argv[nOptionIndex][7];

        }

        // 显示帮助信息

        else if (strncmp(argv[nOptionIndex], "--help", 6) == 0)

        {

            ShowUsage();

            return 0;

        }

        else

        {

            cout << "Options '" << argv[nOptionIndex] << "' not valid. Run '" << argv[0] << "' for details." << endl;

            return -1;

        }

        nOptionIndex++;

    }

 

    cout << "Name is: " << strName << endl;

    cout << "Occupation is: " << strOccupation << endl;

    cout << "Camp is: " << strCamp << endl;

 

    return 0;

}

