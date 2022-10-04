#include "ros/ros.h"
#include "zcx/pathn.h"
#include "zcx.h"

int main(int argc,char **argv)
{
    const char* line1 ="客户端已经就绪！";
    const char* line2 ="操作已完成";
    struct receivedata a;

    a.link_state=false;
    
    do
       {
       //send(line1);
    //   int *link_stste =new int;
    //    ROS_INFO("mode:%lf,x:%lf,y:%lf",a.decison_node,a.decision_x,a.decision_y);
        a=receive();
    //    ROS_INFO("mode:%lf,x:%lf,y:%lf",a.decison_node,a.decision_x,a.decision_y);
        ROS_INFO("mode:%d,x:%lf,y:%lf",a.decison_node,a.decision_x,a.decision_y);

       if(a.link_state==true)
       {
           break;
       }
    }while(1);

    do
    {
       /* code */
        //struct receivedata a=receive();

        struct protogroup b; 

        b = pretreat(a);       

       if(a.decision_flag==true)
       {
           cout<<b.dec.node_number<<endl<<b.dec.x<<endl<<b.dec.y<<endl;
            ros::init(argc,argv,"talker");
	        ros::NodeHandle n;
	        ros::ServiceClient zcx_client = n.serviceClient<zcx::pathn>("mode");
	        //ros::Rate loop_rate(10);

            //int count=0;
            //cout<<"before while"<<endl;
	        //while(count<100)
	        //{
		        zcx::pathn srv;
		        srv.request.mode =b.dec.node_number;
		        srv.request.x =b.dec.x;
		        srv.request.y =b.dec.y;

//		        zcx_pub.publish(msg);
		        
                if(zcx_client.call(srv))
                {
                    ROS_INFO("result:%d",srv.response.result);
                }else{
                    ROS_ERROR("wrong call");
                    return -1;
                }

		        //loop_rate.sleep();
		    //    ++count;
	        //}
            //cout<<"whiled"<<endl;

       }
       if(a.path_flag==true)
       {
           editpart(b);
       }
       if(a.vision_flag==true)
       {
           //还没确定
       }
       if(a.map_flag==true)
       {
           //未完成
       }

        send(line2);

    } while(1);	

}