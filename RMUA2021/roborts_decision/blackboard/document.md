# Blackboard 函数和变量说明
### 变量及其意义
    - chase_acount : 含义
    - 两个搜索路径从外部配置文件加载
    - 撤离目标位置从外部参数文件加载
    - last_enemy_hp_          //敌人剩余血量----------------是否可知？
    - time_diff           //对战时两次检测剩余血量的时间差------------待修改判断值
### 函数及其作用
    - ！！！~~~~~构造函数和析构函数还没写，别忘了~~~~~~！！！
    
    - 还有很多回调函数没有写，根据各方消息修改记录相关参数

    - 功能区待完成函数
        1. GetEnemyYaw()      逃跑姿态可能有问题
        2. LoadBootPositio()  从配置文件中获取出发地位置
        3. IfGetBuff()   判断条件需要根据裁判系统给的变量决定
 
      

    - 待完成回调函数
        1. BuffStatusCallBack
        2. RobotInfoCallBack
            if_punished_;gimbal_punished_;chassis_punished_;
            remain_hp_;remain_bullet_;
        3. EnemyInfoCallBack
            enemy_hp_
        4.EnemyPoseCallBack-------------------motify
        5. GameStatusCallBack
           game_status_
        6. 惩罚
            if_punished_;gimbal_punished_;chassis_punished_;
        7. LoadParam(proto_file_path);

    - pos_reciever.cpp发布的消息是否正确？
