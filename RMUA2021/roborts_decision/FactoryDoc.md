# Goal_factory 函数和变量说明

### 变量及其意义



### 函数及其作用

- 两个取消行为还有可能在冻结上用到，内容待定

- SwingDefend函数的移动方式， 坐标或许可以改为白base_link下的目标坐标，这样可以免去计算距离
- 旋转扫描的初始参数可以设置为当前，不用初始化一个定值应该更好，暂不修改

- ShootGoal 函数可能会导致发射子弹过多，因为你新发送指令，不会改变原来收到的参数，只会新增一个新的发射任务


- CancelGoal()
  CancelGlobalGoal();CancelLocalGoal();
- 