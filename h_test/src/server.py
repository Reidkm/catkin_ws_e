#! /usr/bin/env python
# -*- coding:UTF-8 -*-

import socket  ,os
server = socket.socket()
# host = socket.gethostname()
# client.connect(('localhost',9999))
server.bind(('localhost',9999))
server.listen(2)
while True:
    print('wait client conneting ..')
    recip,addr = server.accept()
    while True:
        print('\033[1;34;0m地址是\033[0m',addr)
        data = recip.recv(1024)
        if not data:
            print('\033[1;38;0m客户端 has lost...\033[0m')
            break
        msg_res = os.popen(data.decode('utf-8')).read()
        print('\033[1;32;0m发送到客户端的内容为：\n\033[0m',msg_res)
        recip.send(str(len(msg_res)).encode())
        recip.send(msg_res.encode())
        print('\033[1;33;0m发送到客户端的内容为：\n\033[0m', msg_res.encode())
        print('\033[1;35;0m发送数据的长度：\033[0m',len(msg_res))
    server.close()
    print('server 运行结束！')
