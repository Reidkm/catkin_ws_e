
#include <iostream>
#include <string>
#include <string.h>
#include <sstream>

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
    stringstream ss;

    string str;

    //std::string strSystime="";
    time_t systime=time(NULL);  

    struct tm *now = localtime(&systime);
    
    int hour = now->tm_hour;
    int min  = now->tm_min;
    int sec  = now->tm_sec;
    int year = now->tm_year + 1900;
    int month = now->tm_mon + 1;
    int day = now->tm_mday;
    cout << year<< "-" << month<< "-" << day 
    <<  " " << hour << ":" << min<< ":" << sec << endl;
 
    //ss<<systime; 
    //ss >> str;
    //cout  << "time is " << str << endl;

    //cout << "time is " << ctime(&systime) << endl;
    //strSystime=ss.str();

    //time_t timep;
   
    //time(&timep); /*获取time_t类型当前时间*/   
    /*转换为常见的字符串：Fri Jan 11 17:04:08 2008*/
    //printf("%s", ctime(&timep));
    //return 0;

 
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
            strName = argv[nOptionIndex]+7 ;
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

