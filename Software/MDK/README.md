# mahonyARHS_G431

#### 介绍
基于双冗余IMU（ICM45686）使用mahony姿态解算  主控STM32G431KBU3

#### 软件架构
配置ICM45686 INT中断读取数据（1600Hz），双IMU数据融合后送往mahony姿态解算（定时器1000Hz），以200Hz输出解算结果，USART1（baudrate 460800）


#### 安装教程

1.  xxxx
2.  xxxx
3.  xxxx

#### 使用说明

1.  连接串口，波特率460800，停止位1 校验位无，ASCII字符显示
2.  xxxx
3.  xxxx

#### 参与贡献

1.  Fork 本仓库
2.  新建 Feat_xxx 分支
3.  提交代码
4.  新建 Pull Request


#### 特技

1.  使用 Readme\_XXX.md 来支持不同的语言，例如 Readme\_en.md, Readme\_zh.md
2.  Gitee 官方博客 [blog.gitee.com](https://blog.gitee.com)
3.  你可以 [https://gitee.com/explore](https://gitee.com/explore) 这个地址来了解 Gitee 上的优秀开源项目
4.  [GVP](https://gitee.com/gvp) 全称是 Gitee 最有价值开源项目，是综合评定出的优秀开源项目
5.  Gitee 官方提供的使用手册 [https://gitee.com/help](https://gitee.com/help)
6.  Gitee 封面人物是一档用来展示 Gitee 会员风采的栏目 [https://gitee.com/gitee-stars/](https://gitee.com/gitee-stars/)
