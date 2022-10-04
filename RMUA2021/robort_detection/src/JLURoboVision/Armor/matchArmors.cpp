/*
*	@Author: Mountain
*	@Date:	 2020.04.24
*	@Brief:  This cpp file define the function 'int matchArmors()' which is used to match lights into armors
*/

#include "../Armor/Armor.h"
#include"../Serial/Serial.h"



/**
* @brief: match lights into armors 将识别到的灯条拟合为装甲板
*/
void ArmorDetector::matchArmors(){
	for (int i = 0; i < lights.size() - 1; i++)
	{
        //int j=i+1; //just ensure every two lights be matched once 从左至右，每个灯条与邻近灯条匹配判断，减少误判//really????
		for (int j = i + 1; j < lights.size(); j++) {
			ArmorBox armor = ArmorBox(lights[i], lights[j]); //construct an armor using the matchable lights 利用左右灯条构建装甲板
            if (armor.isSuitableArmor()) //when the armor we constructed just now is a suitable one,set extra information of armor 如果是合适的装甲板，则设置其他装甲板信息
            {
				armor.l_index = i; //set index of left light 左灯条的下标
				armor.r_index = j; //set index of right light 右灯条的下标
				classifier.getArmorImg(armor);// set armor image 装甲板的二值图
				classifier.setArmorNum(armor);//set armor number 装甲板数字
                 //push into armors 将匹配好的装甲板push入armors中
                if(armor.armorNum != 9)
                {
                    armors.emplace_back(armor);
                }
			}
		}

        eraseCrossLightsArmor();
        eraseErrorRepeatArmor();//delete the error armor caused by error light 删除游离灯条导致的错误装甲板
	}
	if (armors.empty()||armors.size() > 7) {
		state = DetectorState::ARMOR_NOT_FOUND;
        return; //exit function
	} 
	else {
		//cout << "foundArmor" << endl;
		state = DetectorState::ARMOR_FOUND; //else set state ARMOR_FOUND 如果非空（有装甲板）则设置状态ARMOR_FOUND
		return; //exit function
	}
}

/**
 *@brief: set the privious targetArmor as lastArmor and then choose the most valuable armor from current armors as targetArmor
 *			将上一帧的目标装甲板作为lastArmor选择本帧图像中所有装甲板里面价值最大的装甲板作为目标装甲板
 */
void ArmorDetector::setTargetArmor()
{
	if (state == DetectorState::ARMOR_NOT_FOUND) {
		targetArmor = ArmorBox(); //not found armr then set a default armor as lastArmor 如果状态为没有找到装甲板，则将lastArmor设置为默认的ArmorBox
       // printf("!!!!!!!!!!!!:%d\n",targetArmor.armorNum);

    }
	else if (state == DetectorState::ARMOR_FOUND) {
		ArmorBox most_valuable_armor = armors[0]; //mva most valuable armor 最适合打击的装甲板
        for (int i = 1; i < armors.size(); i++) //for circle to select the mva 通过遍历装甲板s获取最佳打击装甲板
        {
            if (armorCompare(armors[i], most_valuable_armor, lastArmor, targetNum))
                most_valuable_armor = armors[i];
        }
//        if ((most_valuable_armor == armors[0])&& most_valuable_armor.armorNum != playerNum && playerNum){
//            most_valuable_armor = ArmorBox();
//            state = DetectorState::ARMOR_NOT_FOUND;
//        }
		targetArmor = most_valuable_armor; //set the mva as the targetArmor of this frame
	}

//	for (int i = 0; i < 4; i++) {
//		targetArmor.armorVertices[i].x += roiNext[2];
//		targetArmor.armorVertices[i].y += roiNext[0];
//	}
//	targetArmor.center.x += roiNext[2];
//	targetArmor.center.y += roiNext[0];

	lastArmor = targetArmor; //first set the targetArmor(of last frame) as lastArmor 将上一帧的targetArmor设置为本帧的lastArmor
}


/**
 *@brief: detect and delete error armor which is caused by the single lightBar 针对游离灯条导致的错误装甲板进行检测和删除
 */
void ArmorDetector::eraseErrorRepeatArmor()
{
	int length = armors.size();
	vector<ArmorBox>::iterator it = armors.begin();
    for (size_t i = 0; i < length; i++) {
        for (size_t j = i + 1; j < length; j++)
		{
            if (
                armors[i].l_index == armors[j].l_index ||
                armors[i].r_index == armors[j].r_index ||
                armors[i].l_index == armors[j].r_index ||
                armors[i].r_index == armors[j].l_index
                )
            {
              if ( armors[i].getDislocationX()>armors[j].getDislocationX() )
              {

//                   printf("############################################?????????????:%f",armors[i].getDislocationX());
//                   printf("############################################?????????????:%f",armors[j].getDislocationX());
           armors.erase(it + i);
           break;
              }
              else if ( armors[i].getDislocationX()<armors[j].getDislocationX())
              {

//                  printf("######################@@@@@@@@@@@@@@@@######################?????????????:%f",armors[i].getDislocationX());
//                  printf("#######################@@@@@@@@@@@@@@@@@@#####################?????????????:%f",armors[j].getDislocationX());
               armors.erase(it + j);
              }
              //armors[i].getAngleDiff() > armors[j].getAngleDiff() ? armors.erase(it + i) : armors.erase(it + j);
				length = armors.size();
				it = armors.begin();

//                printf("######################@@@@@@@@@@@@@@@@######################?????????????:%f",armors[i].getDislocationX());
//                printf("#######################@@@@@@@@@@@@@@@@@@#####################?????????????:%f",armors[j].getDislocationX());

            }


        }
	}
}

void ArmorDetector::eraseCrossLightsArmor()
{
	int length = armors.size();
	vector<ArmorBox>::iterator it = armors.begin();
	for (int i = 0; i < length; i++) {
		int l_index = armors[i].l_index;
		int r_index = armors[i].r_index;
		int l_y = lights[l_index].center.y;
		int r_y = lights[r_index].center.y;
		for (int j = l_index + (int)1; j < r_index; j++) {
			if (lights[j].center.y < MAX(l_y, r_y) + 10 && lights[j].center.y > MIN(l_y, r_y) - 10) {
				armors.erase(it + i); 
				length = armors.size();
				it = armors.begin();
				break;
			}
		}
	}
}


void ArmorDetector::setNumScore(const int & armorNum,const int & targetNum, float & armorScore)
{
    //getBloodVolume(&blood,&vision_receive_);



     if (armorNum != targetNum)
    {
        armorScore += 100000;
    }

}

/**
 *@brief: compare a_armor to b_armor according to their distance to lastArmor(if exit, not a default armor) and their area and armorNum
 *		  比较a_armor装甲板与b_armor装甲板的打击度，判断a_armor是否比b_armor更适合打击（通过装甲板数字是否与目标装甲板数字匹配，装甲板与lastArmor的距离以及装甲板的面积大小判断）
 */
bool ArmorDetector::armorCompare(const ArmorBox & a_armor, const ArmorBox & b_armor, const ArmorBox & lastArmor, const int & targetNum)
{
    float a_score = 0;  // shooting value of a_armor a_armor的打击度
    float b_score = 0;  //shooting value of b_armor b_armor的打击度
    double a_distance = 0, b_distance = 0;

    setNumScore(a_armor.armorNum , playerNum , a_score);
    setNumScore(b_armor.armorNum , playerNum , b_score);
    Point2f CenterPoint = Point2f(640,490);
    a_distance = getPointsDistance(a_armor.center, CenterPoint); //distance score to the lastArmor(if exist) 装甲板距离得分，算负分
    b_distance= getPointsDistance(b_armor.center, CenterPoint); //distance score to the lastArmor(if exist) 装甲板距离得分，算负分



    return a_score < b_score; //judge whether a is more valuable according their score 根据打击度判断a是否比b更适合打击
}
