#! /usr/bin/env python 
# 导入模块
import socket
import random
 
# 创建实例
sk = socket.socket()
 
# 定义需要绑定的ip和端口
ip_port = ("127.0.0.1", 8888)
 
# 绑定监听
sk.bind(ip_port)
 
# 最大连接数
sk.listen(5)
# 不断循环，不断接收数据
while True:
    # 提示信息
    print("正在进行等待接收收据......")
    # 接收数据
    conn, address=sk.accept()
    # 定义信息
    msg = "连接成功！"
    # 返回信息
    # python 3.x以上，网络数据的发送和接收都是byte类型
    # 如果发送的数据是str类型，则需要进行编码
    conn.send(msg.encode())
    # 不断接收客户端发来的信息
    while True:
        # 接收客户端消息
        data = conn.recv(1024)
        # 打印数据
        print(data.decode())
        # 接收退出指令
        if data == b'exit':
            break
        # 处理客户端数据
        conn.send(data)
        # 发送随机数
        conn.send(str(random.randint(1,1000)).encode())
    # 主动关闭
    conn.close()