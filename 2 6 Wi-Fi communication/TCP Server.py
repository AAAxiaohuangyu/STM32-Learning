import socket
import threading
import time


class TCPServer:
    def __init__(self, host='0.0.0.0', port=8080):
        self.host = host
        self.port = port
        self.clients = []
        self.running = True

    def start(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.host, self.port))
        self.socket.listen(5)
        print(f"🚀 TCP服务器启动在 {self.host}:{self.port}")
        print("等待ESP8266连接...")

        # 启动接受连接的线程
        accept_thread = threading.Thread(target=self.accept_connections)
        accept_thread.daemon = True
        accept_thread.start()

        # 主线程处理用户输入
        self.command_interface()

    def accept_connections(self):
        while self.running:
            try:
                client_socket, addr = self.socket.accept()
                print(f"✅ 新连接: {addr}")
                self.clients.append((client_socket, addr))

                # 为新客户端创建线程
                thread = threading.Thread(target=self.handle_client, args=(client_socket, addr))
                thread.daemon = True
                thread.start()

            except Exception as e:
                if self.running:
                    print(f"接受连接错误: {e}")

    def handle_client(self, client_socket, addr):
        try:
            # 发送欢迎消息
            welcome_msg = "Welcome to TCP Server! Commands: LED=ON, LED=OFF, STATUS\r\n"
            client_socket.send(welcome_msg.encode())

            while True:
                data = client_socket.recv(1024).decode('utf-8').strip()
                if not data:
                    break

                print(f"📨 来自 {addr}: {data}")

                # 处理命令
                response = self.process_command(data)
                client_socket.send(f"{response}\r\n".encode())
                print(f"📤 发送响应: {response}")

        except Exception as e:
            print(f"❌ 客户端 {addr} 错误: {e}")
        finally:
            client_socket.close()
            self.remove_client(client_socket)
            print(f"🔌 客户端 {addr} 断开连接")

    def remove_client(self, client_socket):
        self.clients = [client for client in self.clients if client[0] != client_socket]

    def process_command(self, command):
        command = command.upper()
        if "LED=ON" in command:
            return "LED_ON_OK"
        elif "LED=OFF" in command:
            return "LED_OFF_OK"
        elif "STATUS" in command:
            return f"SERVER_STATUS: Clients={len(self.clients)}, Time={time.strftime('%Y-%m-%d %H:%M:%S')}"
        else:
            return f"ECHO: {command}"

    def send_to_client(self, client_index, message):
        """向特定客户端发送消息"""
        if 0 <= client_index < len(self.clients):
            client_socket, addr = self.clients[client_index]
            try:
                client_socket.send(f"{message}\r\n".encode())
                print(f"📤 发送给 {addr}: {message}")
                return True
            except:
                print(f"❌ 发送失败，客户端可能已断开")
                return False
        else:
            print(f"❌ 客户端索引 {client_index} 无效")
            return False

    def list_clients(self):
        """显示所有连接的客户端"""
        print(f"📊 连接中的客户端: {len(self.clients)}")
        for i, (_, addr) in enumerate(self.clients):
            print(f"  {i}. {addr}")

    def command_interface(self):
        """命令行界面，用于手动发送命令给ESP8266"""
        print("\n💡 服务器命令界面:")
        print("  'list' - 显示所有连接的客户端")
        print("  'send <index> <message>' - 向特定客户端发送消息")
        print("  'broadcast <message>' - 向所有客户端广播消息")
        print("  'quit' - 关闭服务器")

        while self.running:
            try:
                cmd = input("\n>>> ").strip()

                if cmd.lower() == 'quit':
                    self.stop()
                    break
                elif cmd.lower() == 'list':
                    self.list_clients()
                elif cmd.startswith('send '):
                    parts = cmd.split(' ', 2)
                    if len(parts) == 3:
                        client_index = int(parts[1])
                        message = parts[2]
                        self.send_to_client(client_index, message)
                    else:
                        print("❌ 用法: send <客户端索引> <消息>")
                elif cmd.startswith('broadcast '):
                    message = cmd[10:]
                    for i in range(len(self.clients)):
                        self.send_to_client(i, message)
                else:
                    print("❓ 未知命令")

            except KeyboardInterrupt:
                self.stop()
                break
            except Exception as e:
                print(f"命令错误: {e}")

    def stop(self):
        print("\n🛑 正在关闭服务器...")
        self.running = False
        for client_socket, addr in self.clients:
            try:
                client_socket.close()
            except:
                pass
        self.socket.close()
        print("✅ 服务器已关闭")


if __name__ == "__main__":
    server = TCPServer()
    try:
        server.start()
    except KeyboardInterrupt:
        server.stop()