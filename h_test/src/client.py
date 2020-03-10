#! /usr/bin/env python
# -*- coding:UTF-8 -*-


from socket import *
import asyncio

client_socket = socket()
host = gethostname()
client_socket.connect(('localhost',9999))

while True:
    cmd = input('输入你要查询的内容:').strip()
    client_socket.send(cmd.encode("utf-8"))
    if len(cmd) == 0:continue
    #接受数据的顺序不能写反
    data_len = client_socket.recv(1024)
    receive_size_len = int(data_len.decode())
 #  client_socket.send('200 ok'.encode())
    receive_data = b''
    receive_len = 0
    #方法一
    while  receive_len < receive_size_len:
        #这里加if判断语句的话，在客户端可以取消注释的一行：client_socket.send('200 ok'.encode())
        if receive_size_len - receive_len > 1024:
            size = 1024
        else:
            size = receive_size_len - receive_len

        data = client_socket.recv(size)
        receive_len += len(data.decode())
        receive_data += data
        print('\033[1;35;01m输出数据为：\n\033[0m',receive_data.decode())

    else:
        print("\033[1;36;01mcmd res receive done...\033[0m")
        break
client_socket.close()
print('\033[1;33;0mclient has over \033[0m')