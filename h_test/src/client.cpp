#include<stdio.h>
#include<sys/types.h>
#include<stdlib.h>
#include<string>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<unistd.h>
#include<iostream>
#include<unistd.h>
#include<string.h>
#include<sstream>


#define ADDR "127.0.0.1" //在本机测试用这个地址，如果连接其他电脑需要更换IP
#define SERVERPORT 8888

using namespace std;

int main()
{
    char buf[100] ;  //stdin buff area

    int socket_fd = socket(AF_INET, SOCK_STREAM,0);
    if(socket_fd == -1)
    {
        cout<<"socket 创建失败："<<endl;
        exit(-1);
    }

    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(SERVERPORT);
    serv_addr.sin_addr.s_addr = inet_addr(ADDR);

    int res = connect(socket_fd,(struct sockaddr*)&serv_addr,sizeof(serv_addr));
    if(res == -1)
    {
        cout<<"bind 链接失败："<<endl;
        exit(-1);
    }
    cout<<"bind 链接成功："<<endl;

    
    string tem_str;
    int tem_count = 0 ;

    while (1)
    {
        /*
        const char *tem_char  = NULL;
        stringstream ss;
        ss << tem_count;
        ss >> tem_str;
        tem_char = ss.str().c_str() ; 
        */
        std::cout << "/* message */" << std::endl;
        write(socket_fd,"HELLO",15);
        std::cout << "1222" << std::endl;
        /*
        fgets(buf, 100, stdin);
        
        if(!strcmp(buf, "q\n") || !strcmp(buf, "Q\n"))
        {
            puts("q pressed\n");
            break;
        }
        */
        tem_count++;
    }

    

    close(socket_fd);

    return 0;
}
