/*
*	@Author: mmllllyyy
*	@Date:	 2021.05.01
*	@Brief:  the function to update shoot state
*/

#include "../Armor/Armor.h"

void ArmorDetector::updateDatas()
{
    if (datas.size() > 260)
		datas.pop_back();

    if(last_jump_index_ < 260)
        last_jump_index_++;

	DataBefore _this;	//该帧数据
	if (state == DetectorState::ARMOR_NOT_FOUND || state == DetectorState::LIGHTS_NOT_FOUND) {
        _this.target_armor_ = ArmorBox();
		datas.insert(datas.begin(), _this);

		return;
	}

	_this.target_armor_ = targetArmor;	//该帧目标装甲板

	int _vis[10][3];	//用于判断目标装甲板相对位置
	memset(_vis, 0, sizeof(_vis));	//初始化

	int _lenght = armors.size();
	for (int i = 0; i < _lenght; i++) {
		int num = armors[i].armorNum;
		_this.armor_quantity_[num]++;
		if (num == targetArmor.armorNum) {
			if (!_vis[num][1]) {		//第一次遍历到该号码装甲板
				_vis[num][1] = 1;

				if (armors[i] == targetArmor) {
					_vis[num][0] = 1;		//标记已遍历到目标装甲板
				}

			}
			else {		//第二次遍历到该号码装甲板
				_vis[num][2] = 1;
				if (_vis[num][0]) {		//前一个遍历到的同号装甲板为目标装甲板
					_this.r_pos_ = RelativePos::LEFT;		//目标装甲板相对位置为左
					//cout << "LEEEEEEEEEEEFT" << endl;
				}
				else {	//当前装甲板即为目标装甲板
					_this.r_pos_ = RelativePos::RIGHT;		//目标装甲板相对位置为右
					//cout << "RIGHTTTTTTTTTTTTTT" << endl;
				}
			}
		}
	}

	if (_vis[targetArmor.armorNum][1] + _vis[targetArmor.armorNum][2] < 2) {	//只遍历到一个同号装甲板
		_this.r_pos_ = RelativePos::ONLY;	//目标装甲板为唯一该车装甲板
		//cout << "ONLYYYYYYYYYYYY" << endl;
	}

	datas.insert(datas.begin(), _this);
}

void inline ArmorDetector::updateJumpCnt()
{
    if (datas[0].follow_car_index_ > 50) {
        datas[0].jump_cnt_ = datas[0].if_jump_ + datas[1].jump_cnt_ - datas[50].if_jump_;
	}
	else {
        datas[0].jump_cnt_ = datas[0].if_jump_ + datas[1].jump_cnt_;
	}
    if(datas[0].jump_cnt_ < 0)
        datas[0].jump_cnt_ = 0;
}

void ArmorDetector::ifFollow()
{
	if (datas.size() < 80) {		//程序刚开始运行
		datas[0].shoot_ = ShootState::LOST;
		datas[0].follow_index_ = 0;
		datas[0].follow_car_index_ = 0;
		datas[0].if_jump_ = 0;
		datas[0].jump_cnt_ = 0;
		return;
	}

	if (shoot == ShootState::LOST) {
		if (!isFoundArmor()) {
			datas[0].shoot_ = ShootState::LOST;
			datas[0].follow_index_ = 0;
			datas[0].follow_car_index_ = 0;
			datas[0].if_jump_ = 0;
			datas[0].jump_cnt_ = 0;
			return;
		}
		int _target_num = targetArmor.armorNum;
		for (int i = 1; i < 4; i++) {		//排除误识别
			if (datas[i].target_armor_.armorNum != _target_num) {
				datas[0].shoot_ = ShootState::LOST;
				datas[0].follow_index_ = 0;
				datas[0].follow_car_index_ = 0;
				datas[0].if_jump_ = 0;
				datas[0].jump_cnt_ = 0;
				return;
			}
		}

		targetNum = _target_num;		//开始跟随该号码装甲板
		shoot = ShootState::FOLLOWING;
		datas[0].shoot_ = ShootState::FOLLOWING;
		datas[0].follow_index_ = 1;
		datas[0].follow_car_index_ = 1;
		datas[0].if_jump_ = 0;
		datas[0].jump_cnt_ = 0;
		stable_armor_ = datas[0].target_armor_;
		cout << "#####start following now#####" << endl;
		return;
	}
	
	//对于切换装甲板和切换目标车辆状态
	int _target_num = targetArmor.armorNum;
	for (int i = 1; i <= 2; i++) {			//防止通信过程中目标发生变化

        if (datas[i].target_armor_.armorNum != _target_num && _target_num && datas[i].target_armor_.armorNum) {	//目标变换或识别不稳定，重新搜索
			shoot = ShootState::LOST;
			datas[0].shoot_ = ShootState::LOST;
			datas[0].follow_index_ = 0;
			datas[0].follow_car_index_ = 0;
			datas[0].if_jump_ = 0;
			datas[0].jump_cnt_ = 0;
			waiting_armor_ = ArmorBox();
			targetNum = 0;
            cout <<_target_num<<" "<<datas[i].target_armor_.armorNum<< "######something is wrong in communication or detecting while switching target set shoot state to lost######" << endl;
			return;
		}

		if (datas[i].shoot_ != shoot) {		//传参未达到2帧
			datas[0].shoot_ = shoot;
			datas[0].follow_index_ = datas[1].follow_index_ + 1;
			datas[0].follow_car_index_ = datas[1].follow_car_index_ + 1;
			datas[0].if_jump_ = 0;
            updateJumpCnt();
			cout << "#####should transfer 2 frames parameters, wait a sec#####" << endl;
			return;
		}

	}

	//下位机正常接受切换目标信息，识别稳定，射击状态改为跟随
	shoot = ShootState::FOLLOWING;
	datas[0].shoot_ = ShootState::FOLLOWING;
	datas[0].follow_index_ = datas[1].follow_index_ + 1;
	datas[0].follow_car_index_ = datas[1].follow_car_index_ + 1;
	datas[0].if_jump_ = 0;
    updateJumpCnt();
	waiting_armor_ = ArmorBox(); 
    stable_armor_ = datas[0].target_armor_;
	cout << "#####switch data transfer success, start following " << targetNum << " now#####" << endl;
	return;
}

void ArmorDetector::ifQuitWaiting()
{
	if (datas.size() < 80) {
		datas[0].follow_index_ = 0;
		datas[0].follow_car_index_ = 0;
		datas[0].if_jump_ = 0;
		datas[0].jump_cnt_ = 0;
		cout << "#####datas is not enought yet#####" << endl;
		return;
	}

	if (!isFoundArmor()) {		//未识别到装甲板时
        if (++lost_cnt_ > 30) {		//失去目标超过20帧，射击状态设为丢失
			cout << "#####target " << targetNum << " lost while waiting#####" << endl;
			targetNum = 0;
			shoot = ShootState::LOST;
			datas[0].shoot_ = ShootState::LOST;
			datas[0].follow_index_ = 0;
			datas[0].follow_car_index_ = 0;
			datas[0].if_jump_ = 0;
			datas[0].jump_cnt_ = 0;
			waiting_armor_ = ArmorBox();
            lost_cnt_ = 0;
		}
		else {
			cout<< "#####target " << targetNum << " waiting "<<lost_cnt_<<" frames#####" << endl;
			datas[0].follow_index_ = datas[0].follow_index_ + 1;
			datas[0].follow_car_index_ = datas[1].follow_car_index_ + 1;
			datas[0].if_jump_ = 0;
			updateJumpCnt();
			waiting_armor_ = stable_armor_;
		}
		return;
	}

	//识别到装甲板时
	int _target_num = targetArmor.armorNum;
	for (int i = 0; i < 4; i++) {
		if (datas[i].target_armor_.armorNum != targetNum) {		//排除误识别
			datas[0].shoot_ = ShootState::WAITING;
			cout << "#####not quiting waiting, didn't found the target we want:" << targetNum << "#####" << endl;
            if (++lost_cnt_ > 40) {
				cout << "#####target " << targetNum << " lost while waiting after waiting "<< lost_cnt_ <<"frames#####" << endl;
				targetNum = 0;
				shoot = ShootState::LOST;
				datas[0].shoot_ = ShootState::LOST;
				datas[0].follow_index_ = 0;
				datas[0].follow_car_index_ = 0;
				datas[0].jump_cnt_ = 0;
				datas[0].if_jump_ = 0;
				waiting_armor_ = ArmorBox();
                lost_cnt_ = 0;
			}
			else {
				cout << "#####target " << targetNum << " waiting " << lost_cnt_ << " frames#####" << endl;
				datas[0].follow_index_ = datas[1].follow_index_ + 1;
				datas[0].follow_car_index_ = datas[1].follow_car_index_ + 1;
				datas[0].if_jump_ = 0;
				updateJumpCnt();
				waiting_armor_ = stable_armor_;
			}
			return;
		}
	}

    lost_cnt_ = 0;
    if (ifSwitchArmor()) {
		shoot = ShootState::ARMOR_SWITCH;
		datas[0].shoot_ = ShootState::ARMOR_SWITCH;
		datas[0].follow_index_ = 0;
		datas[0].follow_car_index_ = datas[1].follow_car_index_ + 1;
		datas[0].if_jump_ = 1;
		updateJumpCnt();
		cout << "####found the previous target " << targetNum << " again while waiting, but armor changed####" << stable_armor_.getDeviationAngle() << " " << targetArmor.getDeviationAngle() << endl;
		waiting_armor_ = ArmorBox();
	}
	else {
		shoot = ShootState::FOLLOWING;
		datas[0].shoot_ = ShootState::FOLLOWING;
		datas[0].follow_index_ = datas[1].follow_index_ + 1;
		datas[0].follow_car_index_ = datas[1].follow_car_index_ + 1;
		datas[0].if_jump_ = 0;
		updateJumpCnt();
		cout << "####found the previous target " << targetNum << " again while waiting, and armor didn't change####" << stable_armor_.getDeviationAngle() << " " <<  targetArmor.getDeviationAngle() << endl;
		waiting_armor_ = ArmorBox();
	}
}

bool ArmorDetector::judgeSwitchCar()
{
//	int _target_num = targetArmor.armorNum;
//	int _switch_cnt = 0; int _switch_pos = 0;		//记录切换目标次数和帧数目录
//	for (int i = 40; i > 1; i--) {
//		if (_target_num != datas[i].target_armor_.armorNum && datas[i].target_armor_.armorNum != 0 && _target_num != 9) {
//			_switch_pos = i;
//			_switch_cnt++;
//		}
//	}

//	if (_switch_cnt > 1) {		//识别不稳定,  不同车之间频繁切换
//		shoot = ShootState::WAITING;
//		datas[0].shoot_ = shoot;
//		datas[0].follow_index_ = datas[1].follow_index_ + 1;
//		datas[0].follow_car_index_ = datas[1].follow_car_index_ + 1;
//		datas[0].if_jump_ = 0;
//		updateJumpCnt();
//		cout << "#####switching car too fast, following " << targetNum << " unstable, switch to waiting#####" << endl;
//		return 1;
//	}

//	if (_switch_pos <= 40 / 2 && _switch_pos > 0) {		//等待更多帧数作为判断
//		shoot = ShootState::WAITING;
//		datas[0].shoot_ = shoot;
//		datas[0].follow_index_ = datas[1].follow_index_ + 1;
//		datas[0].follow_car_index_ = datas[1].follow_car_index_ + 1;
//		datas[0].if_jump_ = 0;
//		updateJumpCnt();
//		cout << "#####waiting for more frames to make decisions if switch to " << _target_num << " while following" << targetNum << "#####" << endl;
//		return 1;
//	}

//	if (_switch_cnt == 1 && _switch_pos > 40/2) {
//		shoot = ShootState::CAR_SWITCH;
//		datas[0].shoot_ = shoot;
//		datas[0].follow_index_ = 0;
//		datas[0].follow_car_index_ = 0;
//		datas[0].if_jump_ = 0;
//		datas[0].jump_cnt_ = 0;
//		cout << "#####target "<< targetNum <<" switched to " << _target_num << "#####" << endl;
//		targetNum = _target_num;
//		return 1;
//	}

//	if (!_switch_pos) {
//		datas[0].follow_car_index_ = datas[1].follow_car_index_ + 1;
//		cout << "#####following "<<targetNum<<", tring to judge if armor switched#####" << endl;
//		return 0;
//	}
    int _target_num = targetArmor.armorNum;
    if(_target_num != targetNum && _target_num){
//        shoot = ShootState::CAR_SWITCH;
//        datas[0].shoot_ = shoot;
//        datas[0].follow_index_ = 0;
//        datas[0].follow_car_index_ = 0;
//        datas[0].if_jump_ = 0;
//        datas[0].jump_cnt_ = 0;
//        cout << "#####target "<< targetNum <<" switched to " << _target_num << "#####" << endl;
//        targetNum = _target_num;
//        return 1;
        shoot = ShootState::WAITING;
        datas[0].shoot_ = shoot;
        datas[0].follow_index_ = datas[1].follow_index_ + 1;
        datas[0].follow_car_index_ = datas[1].follow_car_index_ + 1;
        datas[0].if_jump_ = 0;
        updateJumpCnt();
        cout << "#####waiting for more frames to make decisions if switch to " << _target_num << " while following" << targetNum << "#####" << endl;
        return 1;
    }
    else {
        datas[0].follow_car_index_ = datas[1].follow_car_index_ + 1;
        cout << "#####following "<<targetNum<<", tring to judge if armor switched#####" << endl;
        return 0;
    }
}

bool ArmorDetector::ifSwitchArmor()
{
    vector<Point2f> targetArmorVertices = datas[0].target_armor_.armorVertices;
    vector<Point2f> lastArmorVertices = datas[1].target_armor_.armorVertices;
    cv::Rect targetRect = cv::boundingRect(targetArmorVertices);
    cv::Rect lastRect = cv::boundingRect(lastArmorVertices);
    double IOU = (targetRect & lastRect).area()*1.0/(targetRect | lastRect).area();
    cout<<"######IOU = "<<IOU<<"#######"<<endl;
    if (IOU==0)
    {
        datas[0].shoot_ = shoot;
        datas[0].follow_index_ = 0;
        datas[0].if_jump_ = 1;
        updateJumpCnt();
        waiting_armor_ = ArmorBox();
        stable_armor_ = datas[0].target_armor_;
        cout << "#####armor switched, following " << targetNum << "#####" << endl;
        return 1;
    }
    else {
        datas[0].shoot_ = shoot;
        datas[0].follow_index_ = datas[1].follow_index_ + 1;
        datas[0].if_jump_ = 0;
        updateJumpCnt();
        waiting_armor_ = ArmorBox();
        stable_armor_ = datas[0].target_armor_;
        cout << "#####armor din't change, keep following " << targetNum << "#####" << endl;
        return 0;
    }
}

void ArmorDetector::judgeSwitch()
{
	if (datas.size() < 80 || !isFoundArmor()) {
		shoot = ShootState::WAITING;
		datas[0].shoot_ = shoot;
		datas[0].follow_index_ = datas[1].follow_index_ + 1;
		datas[0].jump_cnt_ = 0;
		datas[0].follow_car_index_ = datas[1].follow_car_index_ + 1;
		datas[0].if_jump_ = 0;
		updateJumpCnt();
		waiting_armor_ = stable_armor_;
		cout << "#####lost target while following, switch to waiting state#####" << endl;
		return;
	}
    cout<<"what nowwwwwwwwwwww"<<endl;
	if (!judgeSwitchCar());
		judgeSwitchArmor();
}

bool ArmorDetector::judgeSpinning()
{
    if (datas[0].jump_cnt_ > 6){
        shoot = ShootState::SPINNING;
        datas[0].shoot_ = shoot;
        datas[0].follow_index_ = datas[1].follow_index_ + 1;
        datas[0].if_jump_ = 0;
        updateJumpCnt();
        waiting_armor_ = ArmorBox();
        stable_armor_ = datas[0].target_armor_;
        cout <<"#####switch to anti spinning mode#####"<<endl;
        return 1;
    }
    else{
        return 0;
    }
}

bool ArmorDetector::judgeSwitchArmor()
{
//	RelativePos _new_pos = datas[0].r_pos_, _old_pos;
//	for (int i = 1; i <= 20; i++) {
//		if (datas[i].shoot_ != ShootState::ARMOR_SWITCH || datas[i].shoot_ == ShootState::FOLLOWING) {
//			_old_pos = datas[i].r_pos_;
//			break;
//		}
//	}

//	if (_old_pos != RelativePos::ONLY && _old_pos == _new_pos) {	//LRFT/RIGHT -> LEFT/RIGHT
//		shoot = ShootState::FOLLOWING;
//		datas[0].shoot_ = shoot;
//		datas[0].follow_index_ = datas[1].follow_index_ + 1;
//		datas[0].if_jump_ = 0;
//		updateJumpCnt();
//		waiting_armor_ = ArmorBox();
//		stable_armor_ = datas[0].target_armor_;
//		cout << "#####armor din't change, keep following " << targetNum << "#####" << endl;
//		return 0;
//	}

//	if (_old_pos == RelativePos::ONLY && _new_pos != RelativePos::ONLY) {	//OLNY -> LEFT/RIGHT
//		shoot = ShootState::FOLLOWING;
//		datas[0].shoot_ = shoot;
//		datas[0].follow_index_ = datas[1].follow_index_ + 1;
//		datas[0].if_jump_ = 0;
//		updateJumpCnt();
//		waiting_armor_ = ArmorBox();
//		stable_armor_ = datas[0].target_armor_;
//		cout << "#####armor din't change, keep following " << targetNum << "#####" << endl;
//		return 0;
//	}

//	if (_old_pos != _new_pos && _old_pos != RelativePos::ONLY && _new_pos != RelativePos::ONLY) {		//LEFT/RIGHT -> RIGHT/LEFT
//		shoot = ShootState::ARMOR_SWITCH;
//		datas[0].shoot_ = shoot;
//		datas[0].follow_index_ = 0;
//		datas[0].if_jump_ = 1;
//		updateJumpCnt();
//		waiting_armor_ = ArmorBox();
//		stable_armor_ = datas[0].target_armor_;
//		cout << "#####armor switched, following " << targetNum << "#####" << endl;
//		return 1;
//	}

//	if (_old_pos != RelativePos::ONLY && _new_pos == RelativePos::ONLY) {	//LEFT/RIGHT -> ONLY
//		float _old_angle = datas[1].target_armor_.getDeviationAngle();
//		float _new_angle = datas[0].target_armor_.getDeviationAngle();
//        cout << _old_angle<<" "<<_new_angle<<endl;
//		if (abs(_old_angle - _new_angle) > 5) {
//			shoot = ShootState::ARMOR_SWITCH;
//			datas[0].shoot_ = shoot;
//			datas[0].follow_index_ = 0;
//			datas[0].if_jump_ = 1;
//			updateJumpCnt();
//			waiting_armor_ = ArmorBox();
//			stable_armor_ = datas[0].target_armor_;
//			cout << "#####armor switched, following " << targetNum << "#####" << endl;
//			return 1;
//		}

//		datas[0].shoot_ = shoot;
//		datas[0].follow_index_ = datas[1].follow_index_ + 1;
//		datas[0].if_jump_ = 0;
//		updateJumpCnt();
//		waiting_armor_ = ArmorBox();
//		stable_armor_ = datas[0].target_armor_;
//		cout << "#####armor din't change, keep following " << targetNum << "#####" << endl;
//		return 0;
//	}

//	if (_old_pos == RelativePos::ONLY && _new_pos == _old_pos) {	//ONLY -> ONLY
//		float _old_angle = datas[1].target_armor_.getDeviationAngle();
//		float _new_angle = datas[0].target_armor_.getDeviationAngle();
//        cout << _old_angle<<" "<<_new_angle<<endl;
//		if (abs(_old_angle - _new_angle) > 5) {
//			shoot = ShootState::ARMOR_SWITCH;
//			datas[0].shoot_ = shoot;
//			datas[0].follow_index_ = 0;
//			datas[0].if_jump_ = 1;
//			updateJumpCnt();
//			waiting_armor_ = ArmorBox();
//			stable_armor_ = datas[0].target_armor_;
//			cout << "#####armor switched, following " << targetNum << "#####" << endl;
//			return 1;
//		}

//		datas[0].shoot_ = shoot;
//		datas[0].follow_index_ = datas[1].follow_index_ + 1;
//		datas[0].if_jump_ = 0;
//		updateJumpCnt();
//		waiting_armor_ = ArmorBox();
//		stable_armor_ = datas[0].target_armor_;
//		cout << "#####armor din't change, keep following " << targetNum << "#####" << endl;
//		return 0;
//	}
    if(ifSwitchArmor()){
        shoot = ShootState::ARMOR_SWITCH;
        //cout<<"#####switch armor######"<<endl;
    }
    judgeSpinning();
}

void ArmorDetector::shootSpinning() 
{
    int index = 0;
    while(datas[index].target_armor_.l_index == -1){
        index++;
    }
//    targetArmor = datas[index].target_armor_;
//    last_jump_index_ = 0;
    if(ifSwitchArmor()&&state == DetectorState::ARMOR_FOUND){
        last_jump_index_ = 0;
        predict_spinning_.x = (datas[0].target_armor_.center.x + datas[index].target_armor_.center.x)/2;
        predict_spinning_.y = (datas[0].target_armor_.center.y + datas[index].target_armor_.center.y)/2;
    }
    else {
        predict_spinning_.y = datas[index].target_armor_.center.y;
        predict_spinning_.x = datas[index].target_armor_.center.x;
    }

    circle(src, predict_spinning_, 2, Scalar(255, 255, 0), 2, 8, 0);
    //Mat asedf = Mat::zeros(1024, 1280, CV_8UC3);
//    cout<<"sssssssssssssssssss"<<targetArmor.center<<endl;
//    cout<<"ffffffffffffffffff"<<targetArmor.armorVertices<<endl;
//    for (size_t i = 0; i < 4; i++)
//    {
//        line(src, targetArmor.armorVertices[i], targetArmor.armorVertices[(i + 1) % 4], Scalar(0, 255, 255), 2, 8, 0);
//    }
    imshow("asdf", src);
    if (datas[0].jump_cnt_ < 6)
    {
        shoot = ShootState::FOLLOWING;
        datas[0].shoot_=shoot;
    }
}

void ArmorDetector::updateShootState()
{
	updateDatas();
	if (shoot == ShootState::CAR_SWITCH || shoot == ShootState::ARMOR_SWITCH || shoot == ShootState::LOST) {
		ifFollow();		//是否开始跟随
		cout << 1111111 << endl;;
	}
	else if (shoot == ShootState::WAITING) {
		ifQuitWaiting();		//是否退出等待
		cout << 2222222 << endl;;
	}
	else if (shoot == ShootState::FOLLOWING) {
        judgeSwitch();
		cout << 44444444 << endl;;
	}
    if (shoot == ShootState::SPINNING){
        if(judgeSwitchCar()){
            goto out;
        }
        shootSpinning();
        cout<< 555555555 << endl;
    }
    out:
    cout<<"####lost cnt "<<lost_cnt_<<"########"<<endl;
    cout << "####till now we've followed " << datas[0].follow_index_ << " frames#####" << endl;
    cout<<"#####following car "<<targetNum<<" "<<datas[0].follow_car_index_<<"######"<<endl;
    cout<<"#####if this frame jump "<<datas[0].if_jump_<<"######"<<endl;
    cout << "####there have been " << datas[0].jump_cnt_ << " jump from 240 frames before#####" << endl;
    cout<<"######last jump index is "<<last_jump_index_<<"######"<<endl;
    cout<<endl;
}

Point2f ArmorDetector::getSpinningPoint()
{
    return predict_spinning_;
}
