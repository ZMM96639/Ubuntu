### 1. ROS
##### a. [简易安装](https://blog.csdn.net/KIK9973/article/details/118755045)
```
wgethttp://fishros.com/install-Ofishros&&chmod+xfishros&&sudo./fishros
```

##### b. 其它工具
```
sudo apt install rospack-tools python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install ros-melodic-map-server
sudo apt install ros-melodic-bfl
sudo apt install libceres-dev
sudo apt install ros-melodic-move-base-msgs
sudo apt install ros-melodic-joint-state-publisher-gui ros-melodic-ddynamic-reconfigure
sudo apt install terminator #终端工具，分栏方便
```

##### c. 检查安装成功
```
roscore
```

### 2. git配置
```
sudo apt install git
# 配置公钥
git config --global user.email "xxx.com.cn"
git config --global user.name "xxx"
```

### 3. [输入法](https://shurufa.sogou.com/linux/guide)
```
sudo apt install fcitx
wget http://cdn2.ime.sogou.com/dl/index/1571302197/sogoupinyin_2.3.1.0112_amd64.deb
```
注：搜狗拼音法（18.04只能下载2.3版本（或者其他2.x版本）的，不要下载官网3.x或者4.x版本，不然安装了也用不了）

### 4. Ubuntu安装免费版Typora
```
# 安装
wget https://file.babudiu.com/f/yXCL/Typora_Linux_0.11.18_amd64.deb
sudo dpkg -i Typora_Linux_0.11.18_amd64.deb

# 卸载
dpkg -l | grep xxx
sudo apt purge xxx
```

### 5. 高效软件推荐
```
balenaEtcher      # 制作U盘启动盘
Flameshot         # 截图贴图
Meld              # 文件比较
Xmind             # 思维导图
Peek              # 录制gif
Clash for Windows # vpn(魔戒)
```