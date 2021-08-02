# 服务变量

请求:

`bool req`

回复:

`bool res`

# 测试使用说明

1. 启动roscore，在下面其他终端运行指令时都要先source

   ```bash
   roscore
   
   source ./devel/setup.bash
   ```

   

2. 给transfer_txt.py和receive_txt.py添加运行权限

   ```bash
   chmod +x transfer_txt.py
   chmod +x receive_txt.py
   ```

3. 运行发送txt文档的节点，

   ```bash
   rosrun transfer transfer_txt.py
   rosrun transfer receive_txt.py
   ```

4. 开启另一个终端，并在终端中查询

   ```bash
   # 查询是否有服务打开，并查看服务是否为transmit_txt
   rosservice list
   # 输入以下命令既可以传递文件,可能是TRUE | True，记不清了，尽量用tab补全来确定是否正常
   rosservice call /transmit_txt ture
   
   ```

   