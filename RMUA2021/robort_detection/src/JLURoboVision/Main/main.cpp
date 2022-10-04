#include"../Armor/Armor.h"
#include"../GxCamera/GxCamera.h"
#include"../AngleSolver/AngleSolver.h"
#include"../General/General.h"
//#include<X11/Xlib.h>
#include <errno.h>
//#include <unistd.h>
//#include <syslog.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
//#include <sys/wait.h>
#include <signal.h>

using std::thread;


// muti-threads control variables
mutex   Globalmutex;                        // C++11 mutex
condition_variable GlobalCondCV;            // C++11 condition variable
condition_variable SerialSendCond;
condition_variable SerialReceiveCond;
bool imageReadable = false;              // threads conflict due to image-updating
bool sendNow = false;
bool receiveNow = false;
Mat src = Mat::zeros(600, 800, CV_8UC3);   // Transfering buffer
mutex SerialReceiveLock;
mutex SerialSendLock;
Send_to_embedded send_data;
vision_re receive_data;

//void forkChildProcess(int){
//    int status =0;
//    int pid=wait(&status);
//    if(pid<0){
//        printf("error: %s\n",strerror(errno));
//        return;
//    }
//
//    if (WIFSIGNALED(status)){
//        int signalNum=WTERMSIG(status);
//        printf("Child process was killed by signal num: %d\n",signalNum);
//    }
//    if (WCOREDUMP(status)){
//        printf("Chiled process core dump file generated\n");
//
//    }
//
//    sleep(3);
//    pid = fork();
//    if (pid==0){
//        printf ("FOrk new child process\n");
//        //ChildProcessFunc();
//        (void)init();
//    }
//
//
//}
//
//bool initwatchDog(){
//    int pid = fork();
//    if (pid){
//        while(true){
//            assert(signal(SIGCHLD,forkChildProcess)!=SIG_ERR);
//            pause();
//        }
//    }
//    else if (pid<0){
//        return false;
//    }
//
//    return true;
//}
//int init(){
//    printf("pthread begin\n");
//    int ret = 0;
//    ret = childinit();
//    if (true!= ret){
//        printf("init: childinit Fail!\n");
//        return false;
//    }
//    return true;
//}
//
//int childinit(){
//    int iRet=0;
//    pthread_t Thread_ID;
//    iRet = pthread_create(&Thread_ID,NULL,ChildProcessFunc,NULL);
//    if (iRet!=0){
//        printf("childinit: childProcessFunc faild!\n");
//        return false;
//    }
//    return true;
//}
//
//
//
//
//void *ChildProcessFunc(void *)
//{
//	// camera image updating thread
//	thread(imageUpdatingThread).detach();
//	// armor detecting thread
//	thread(armorDetectingThread).detach();
//	// main thread
//	while (true)
//	{
//		char chKey = getchar();
//		if (chKey == 'Q' || chKey == 'q')
//			return 0;
//	}
//	return 0;
//}
//
//
//int main(){
//    //int ret = 0;
//    printf("Main pid: %d\n",getpid());
//   bool ret=initwatchDog();
//    if(!ret){
//        printf("Init watch dog failed\n");
//        return -1;
//    }
//    printf("Init watch dog sucess...\n");
//   //ChildProcessFunc();
//    (void)init();
//    return 0;
//}

int main()
{
	// camera image updating thread
    thread(imageUpdatingThreadCamera).detach();
	//thread(imageUpdatingThreadLocal).detach();
	// armor detecting thread
    thread(armorDetectingThread).detach();


	// main thread
	while (true)
	{
		char chKey = getchar();
		if (chKey == 'Q' || chKey == 'q')
			return 0;
	}
	return 0;
}
