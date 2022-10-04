# Action_node

## 类说明

    **重要说明，写结点的时候，不要让节点中掺入行为树将要使用的判断条件，容易引起混乱**

- BackBootArea 回到启动区，游戏结束或者其他必要条件时回到启动区
- BeginningAction 为第一种开场动作，2号车去拿子弹补给，1号车跟随2号车
- BeginningActionTwo 为第二种开场动作，2号车去拿子弹补给，1号车去追击敌人
- ChaseAction 追击敌人
- ChassisLimited 底盘受限
- DefendAction 摆动防御
- Escape 逃跑节点 （趋向逃离目标并且在途中检测到敌人的时候采取适当攻击或者更改逃跑路线）
- FollowAction 跟随我方另一台车，以便合作
- FrozeAction 冻结节点，游戏正式开始之前一切保持不动
- GainBloodAction 获取子弹补给（buff区刷新之后，趋向适当的buff位置以获取buff）
- GainBulletAction 获取回血buff（buff区刷新之后，趋向适当的buff位置以获取buff）
- SearchAction 在预定的搜索路线中进行搜索
- ShootAction 射击敌人
- ShootWithDefend 防御敌人
- TurnToDetectedDirection 转向受攻击方向
- WaitBuffRefresh 在buff区刷新前，离开buff区到不远处，伺机等待刷新
- IsUnderPunish()判断是否受到惩罚，如果不能在进入节点之前先调用SetUnderPunish()，则云台受限和底盘受限需要改


## 粗略预计待完成节点

### 据情况改动


## 一些问题

- 追击不应该用时间来判断是否追上
