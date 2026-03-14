
#include "LQ_TCP_Client.hpp"

/**
 * @brief TCP客户端类的无参构造函数
 * @details 初始化TCP客户端对象，不建立连接
 */
TCP_Client::TCP_Client()
{
}

/*!
 * @brief   TCP 客户端类的有参构造函数
 * @param   ip   : 连接服务器的 IP 地址
 * @param   port : 连接服务器的端口号
 */
TCP_Client::TCP_Client(const string &ip, uint16_t port)
{
    this->TCP_client_init(ip, port);
}


/**
 * @brief 初始化TCP客户端并连接到指定服务器
 * 
 * 该函数创建TCP套接字，配置服务器地址信息，并尝试连接到指定的服务器。
 * 如果连接成功，返回0；如果失败，返回相应的错误码。
 * 
 * @param ip 服务器IP地址字符串
 * @param port 服务器端口号
 * @return uint8_t 返回值说明：
 *         -  0: 连接成功
 *         - -1: 套接字创建失败
 *         - -2: IP地址转换失败
 *         - -3: 连接服务器失败
 */
uint8_t TCP_Client::TCP_client_init(const string &ip, uint16_t port)
{
    // 创建TCP套接字
    this->sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (this->sockfd == -1)
    {
        perror("tcp_cli_socket:");
        return -1;
    }
    // 设置地址复用选项
    int val = 1;
    setsockopt(this->sockfd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val));
    // 准备服务器地址结构
    struct sockaddr_in seraddr;
    memset(&seraddr, 0, sizeof(seraddr));
    seraddr.sin_family = AF_INET;   // 设置地址族为IPv4
    seraddr.sin_port = htons(port); // 设置服务器端口，htons转换字节序
    // 设置服务器IP地址
    if (inet_aton(ip.c_str(), &seraddr.sin_addr) == 0)
    {
        perror("tcp_cli_inet_aton:");
        close(this->sockfd);
        return -2;
    }
    // 连接服务器
    if (connect(this->sockfd, (struct sockaddr *)&seraddr, sizeof(seraddr)) == -1)
    {
        perror("tcp_cli_connect:");
        close(this->sockfd);
        return -3;
    }

    return 0;
}

/**
 * @brief TCP客户端发送信息函数
 * 
 * @param buf 要发送的数据缓冲区指针
 * @param len 要发送的数据长度
 * @return ssize_t 成功返回发送的字节数，失败返回-1
 * 
 * @note 使用示例：cli.TCP_Send(buf, sizeof(buf));
 */
ssize_t TCP_Client::TCP_Send(const void *buf, size_t len)
{
    ssize_t ret = send(this->sockfd, buf, len, 0);
    if (ret < 0)
    {
        perror("TCP_cli_send:");
    }
    return ret;
}

/**
 * @brief TCP客户端接收数据函数
 * @param buf 接收数据的缓冲区指针
 * @param len 需要接收的数据长度
 * @return 成功返回实际读取的字节数，失败返回-1
 * @note 该函数通过套接字接收数据，失败时会打印错误信息
 */
ssize_t TCP_Client::TCP_Recv(void *buf, size_t len)
{
    ssize_t ret = recv(this->sockfd, buf, len, 0);
    if (ret < 0)
    {
        perror("TCP_cli_recv:");
    }
    return ret;
}


/**
 * 获取TCP客户端的套接字描述符
 * 
 * @return 返回当前TCP客户端的套接字描述符
 */
int TCP_Client::TCP_Get_socket()
{
    return this->sockfd;
}

/**
 * 析构函数，关闭TCP客户端套接字
 * 
 * 释放TCP客户端资源，关闭已建立的套接字连接
 */
TCP_Client::~TCP_Client()
{
    close(this->sockfd);
}
