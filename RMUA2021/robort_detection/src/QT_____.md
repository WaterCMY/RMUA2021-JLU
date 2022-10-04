# QT程序自启动

环境 Ubuntu18.04  Qt5

1，项目在release模式下编译一遍，在文件夹中可以找到可执行程序，文件夹里创建一个***.desktop的文件（我的命名为demo.desktop）然后在这个文件中编辑。

```
vim demo.desktop
```

linux下编译后并不会生成 .exe的可执行程序，linux下是application/....类型的文件，如果不能打开

解决办法：在程序的.pro文件里加上

```
QMAKE_LFLAGS += -no-pie
```

2，在编辑界面中输入如下信息，保存并退出。路径为可执行程序的路径（即可直接双击打开运行程序的路径）。

```c++
[Desktop Entry]
Version=1.0.1
Name=demo
Exec=/home/Downlodas/JLU_Demo/JLU_Demo
Type=Application
```

3，对上面的文件增加权限。

```
chmod 755 demo.desktop
```

增加权限后的文件双击就可打开程序了。

4，复制该文件到系统自启动的启动器文件下。

```
sudo cp demo.desktop /etc/xdg/autostart
```

电脑重启后程序就可以自动启动了，如果想删除开机自启，在autostart文件下删除demo.desktop文件就行了。