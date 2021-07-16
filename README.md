# Autonomous-vehicles-implementation-L1
This is a item about Autonomous Vehicles implementation on L1. It has been only tested on our platform(Ubuntu 20.04, STM32F407IGT6)

## Attention

The reason to build this repository is my post graduation project. It only recorded something about self-driving car, our research projects' group car, implementation on L1. The purpose of doing this is to make car self move and stop. I also hope it can Identify person so that I can make it stop the car when it move too close to people.

My aim is to build a end-to-end autonomous driving system. To realize this objectives, all of PC and PLC control is built on ROS Noetic and STM32.

It may not help you, beacuse these packages only tested on our car.

---

Vehicle hardware information:

**PC**:

| CPU  | I7-3517U |
| :--: | :------: |
| RAM  |   12G    |
|  HD  |   256G   |
| GPU  |   None   |

**PLC**:

| Development Boards |   硬石科技    |
| :----------------: | :-----------: |
|        CPU         | STM32F407IGT6 |

## Classification

### PC

1. [STM32和ROS noetic串口通信](https://github.com/sunyuan789/Autonomous-vehicles-implementation-L1/tree/main/usart)**USART**功能包要配合**serial-master**功能包一起使用，实际上就是noetic-serial功能包，但是使用sudo install的方式并不能从官网直接下载，因此需要从github源码上直接使用。

### PLC




