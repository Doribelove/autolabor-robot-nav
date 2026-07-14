import pika
import sys


# ====================== 配置项 ======================
RABBITMQ_HOST = "39.98.47.163"    # 你的 RabbitMQ 服务器 IP/域名
RABBITMQ_PORT = 5672         # AMQP 默认端口
RABBITMQ_USER = "caacsriUser"      # 用户名（远程连接不要用 guest，默认禁止远程访问）
RABBITMQ_PWD  = "caacsriUser"  # 密码
QUEUE_NAME    = "collection_vehicle"     # 队列名称，必须与发送端完全一致
VIRTUAL_HOST  = "/"              # 虚拟主机，默认是 /
# ===================================================

def main():
    # 1. 构造登录凭证
    credentials = pika.PlainCredentials(RABBITMQ_USER, RABBITMQ_PWD)

    # 2. 建立连接
    connection_params = pika.ConnectionParameters(
        host=RABBITMQ_HOST,
        port=RABBITMQ_PORT,
        virtual_host=VIRTUAL_HOST,
        credentials=credentials,
        heartbeat=600,  # 心跳保活，防止连接被防火墙断开
        blocked_connection_timeout=300
    )
    connection = pika.BlockingConnection(connection_params)
    channel = connection.channel()

    # 3. 声明队列（幂等：不存在则创建，存在则校验属性）
    # durable=True 表示队列持久化，必须和生产者保持完全一致
    channel.queue_declare(queue=QUEUE_NAME, durable=True)

    # 4. 定义消息处理回调函数
    def message_callback(ch, method, properties, body):
        """
        ch: 通道对象
        method: 交付信息（包含 delivery_tag 等）
        properties: 消息属性
        body: 消息内容（bytes 类型）
        """
        try:
            # 解码消息（根据实际编码调整）
            message = body.decode("utf-8")
            print(f"[√] 收到消息: {message}")

            # ========== 在这里写你的业务处理逻辑 ==========
            # 例如：解析数据、写入数据库、调用接口等

            # 5. 手动确认消息已成功处理
            # 只有执行这一步，RabbitMQ 才会把消息从队列中删除
            ch.basic_ack(delivery_tag=method.delivery_tag)

        except Exception as e:
            print(f"[×] 处理消息失败: {e}")
            # 处理失败时，拒绝消息并重入队列（requeue=True）
            ch.basic_nack(delivery_tag=method.delivery_tag, requeue=True)

    # 6. 绑定消费者到队列
    # auto_ack=False 关闭自动确认，必须手动调用 basic_ack
    channel.basic_consume(
        queue=QUEUE_NAME,
        on_message_callback=message_callback,
        auto_ack=False
    )

    print(f"[*] 已连接 RabbitMQ，等待队列 {QUEUE_NAME} 的消息...")
    print("[*] 按 Ctrl+C 退出")

    # 7. 启动阻塞消费循环
    channel.start_consuming()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[!] 手动中断，退出消费")
        sys.exit(0)
    except Exception as e:
        print(f"[!] 连接或运行异常: {e}")
        sys.exit(1)