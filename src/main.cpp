//いつもの//
#include <stdio.h>
//opencv//
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//ROS//
#include <ros/ros.h>
//時間関係//
#include "time.h"
#include <sys/time.h>
//math.h//
#include <math.h>
//キー入力//
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
//クラス分けしたやつ//
#include "scan.cpp"
#include "create_map.cpp"

static struct timeval microsSource;
static int64_t offsetSeconds;
static int64_t offsetmicros;

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

int timer_setup(){
    gettimeofday(&microsSource, NULL);
    offsetSeconds=microsSource.tv_sec;
    offsetmicros=microsSource.tv_usec;
    return 0;
}
int64_t micros() {
    gettimeofday(&microsSource, NULL);
    return (microsSource.tv_sec-offsetSeconds)*(int64_t)1000000+(microsSource.tv_usec-offsetmicros);
}

/* 画像を画像に貼り付ける関数
void paste(cv::Mat dst, cv::Mat src, int x, int y, int width, int height) {
	cv::Mat resized_img;
	cv::resize(src, resized_img, cv::Size(width, height));

	if (x >= dst.cols || y >= dst.rows) return;
	int w = (x >= 0) ? std::min(dst.cols - x, resized_img.cols) : std::min(std::max(resized_img.cols + x, 0), dst.cols);
	int h = (y >= 0) ? std::min(dst.rows - y, resized_img.rows) : std::min(std::max(resized_img.rows + y, 0), dst.rows);
	int u = (x >= 0) ? 0 : std::min(-x, resized_img.cols - 1);
	int v = (y >= 0) ? 0 : std::min(-y, resized_img.rows - 1);
	int px = std::max(x, 0);
	int py = std::max(y, 0);

	cv::Mat roi_dst = dst(cv::Rect(px, py, w, h));
	cv::Mat roi_resized = resized_img(cv::Rect(u, v, w, h));
	roi_resized.copyTo(roi_dst);
}

// 画像を画像に貼り付ける関数（サイズ指定を省略したバージョン）
void paste(cv::Mat dst, cv::Mat src, int x, int y) {
	paste(dst, src, x, y, src.rows, src.cols);
}*/



int
main(int argc, char *argv[])
{
  cv::Mat back = cv::Mat::zeros(1817,1817, CV_8UC3);
  timer_setup();
  int64_t scanT = micros();
  int64_t readT = micros();
  create_map create;
  create.setup();

  float x[1440];
  float y[1440];
  float rev_x[1440];
  float rev_y[1440];
  float dis[1440]={};
  float old_dis[1440]={};
  float rad[1440]={};
  int step = 0;
  int flag;
  int hit=0;
  int d_hit=0;
  int good_data = 0;
  float good_x[1440];
  float good_y[1440];
  int reset_flag = 0;
  int loop_cnt = 0;


  //データの数//
  int num[11]={};
  //データの数(+)//
  int  p_num[11]={};
  //データの数(+)//
  int  m_num[11]={};
  //データの最大
  float max[11]={};
  //データの最小
  float min[11]={};
  //（誤差）データの合計//
  float total[11]={};
  //データ(+)の合計//
  float p_total[11]={};
  //データ(-)の合計//
  float m_total[11]={};
  //データの平均//
  float average[11]={};
  //データ(+)の平均//
  float p_average[11]={};
  //データ(-)の平均//
  float m_average[11]={};
  //誤差の格納//
  float diff_box[11][1080];
  //一時的に格納//
  float tmp[11]={};
  //中央値//
  float med[11]={};
  //偏差//
  float devi[11]={};
  //分散//
  float disp[11]={};
  //標準偏差//
  float std_devi[11]={};
  float old_ave[11]={};

  int get_num;


  int q;
  int w;
  int i;
  int j;
  int cnt_flag=1;
  int first_cnt=0;
  int key_num;
  int old_key_num;

  int oldave_flag=1;
  int old_hit=0;
  int hit_cnt[11]={};
  for(q=0;q<11;q++){
    for(w=0;w<1080;w++){
      diff_box[q][w]=0.0;
    }
  }

  ros::init(argc, argv, "scan");
  scan scan;
  cv::Mat coat_img;
  cv::Mat src_img=cv::imread("/home/kuge/catkin_ws/src/opencv_test/src/coat_numbers.jpg", 1);
  cv::resize(src_img,coat_img,cv::Size(), 0.5,0.5);
  /*printf("場所を選んでキー入力\n右上=1\n右下=2\n左上=3\n右真ん中上=4\n右真ん中下=5\n右真ん中下の下=6\n右下の上=7\n右下の下=8\n");
  scanf("%d",&flag);
  if(flag < 1 || flag>8){
    printf("エラー.....無効な入力です\n");
    return 0;
  }*/
  if(reset_flag==0){
    hit = 0;
    printf("場所を選んでキをー入力してEnterを押してください\n");
    printf("\n");
    printf("その後は対応した数字を押すとLRFの場所が変わります\n");
    printf("\n");
    printf("キー入力で線のあたっている場所の情報を切り替えます\n");
    cv::namedWindow("LRFの場所", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
    cv::imshow("LRFの場所", coat_img);
    cv::waitKey(1);
    scanf("%d",&flag);
    reset_flag = 1;
    if(flag < 1 || flag>8){
      printf("エラー.....無効な入力です\n");
      return 0;
    }
  }

  switch (flag){
    case 1:
    create.origin(2250,2250);
    create.set_rev_rad(225.0);
    break;

    case 2:
    create.origin(2250,15920);
    create.set_rev_rad(315.0);
    break;

    case 3:
    create.origin(7240,2250);
    create.set_rev_rad(135.0);
    break;

    case 4:
    create.origin(7240,8920);
    create.set_rev_rad(45.0);
    break;

    case 5:
    create.origin(7240,12400);
    create.set_rev_rad(135.0);
    break;

    case 6:
    create.origin(7240,13970);
    create.set_rev_rad(45.0);
    break;

    case 7:
    create.origin(10500,14350);
    create.set_rev_rad(135.0);
    break;

    case 8:
    create.origin(10500,15920);
    create.set_rev_rad(45.0);
    break;

    default:
    break;
  }
  while(ros::ok()){
    if (kbhit()) {
      if(hit > 9){
        d_hit = 0;
      }
      //printf("'%c'を押しました。\n", getchar());
      //getchar();
      get_num=getchar();
      d_hit+=1;

    }
    ros::spinOnce();

    if(micros() - scanT > 10000){
      switch (get_num){
        case '1':
        create.origin(2250,2250);
        create.set_rev_rad(225.0);
        key_num=1;
        break;

        case '2':
        create.origin(2250,15920);
        create.set_rev_rad(315.0);
        key_num=2;
        break;

        case '3':
        create.origin(7240,2250);
        create.set_rev_rad(135.0);
        key_num=3;
        break;

        case '4':
        create.origin(7240,8920);
        create.set_rev_rad(45.0);
        key_num=4;
        break;

        case '5':
        create.origin(7240,12400);
        create.set_rev_rad(135.0);
        key_num=5;
        break;

        case '6':
        create.origin(7240,13970);
        create.set_rev_rad(45.0);
        key_num=6;
        break;

        case '7':
        create.origin(10500,14350);
        create.set_rev_rad(135.0);
        key_num=7;
        break;

        case '8':
        create.origin(10500,15920);
        create.set_rev_rad(45.0);
        key_num=8;
        break;

        default:
        break;
      }
      hit=d_hit;
      if (hit>10) {
        hit=10;
      }
      scanT = micros();
      create.show();
      good_data = 0;
      for(step = 0;step < 1080;step=step+1){
        old_dis[step] = dis[step];
        rad[step] = (M_PI/180)*(step*0.25);
        if(fabsf(old_dis[step]-dis[step])>0.02){
          dis[step]=0.0;
        }

        else{dis[step] = scan.scan_val(step);}

        create.create(dis[step],rad[step],step,flag);

        switch(create.return_flag()){
          /*case 1:
          num[1]+=1;
          diff_box[create.return_flag()][num[1]]=create.return_val_x();*/
          case 1:
            num[1]+=1;
            diff_box[create.return_flag()][num[1]]=create.judge();
            if(max[1]<create.judge()){max[1] = create.judge();}
            if(min[1]>create.judge()){min[1] = create.judge();}
            total[1]= total[1]+create.judge();
            if(create.judge()>=0){
              p_num[1]+=1;
              p_total[1]=p_total[1]+create.judge();
            }else{
              m_num[1]+=1;
              m_total[1]=m_total[1]+create.judge();
            }
            average[1] = total[1] / (float)num[1];
            p_average[1]=p_total[1]/ (float)num[1];
            m_average[1]=m_total[1]/ (float)num[1];
            break;
          case 2:
            num[2]+=1;
            diff_box[create.return_flag()][num[2]]=create.judge();
            if(max[2]<create.judge()){max[2] = create.judge();}
            if(min[2]>create.judge()){min[2] = create.judge();}
            total[2]= total[2]+create.judge();
            if(create.judge()>=0){
              p_num[2]+=1;
              p_total[2]=p_total[2]+create.judge();
            }else{
              m_num[2]+=1;
              m_total[2]=m_total[2]+create.judge();
            }
            average[2] = total[2] / (float)num[2];
            p_average[2]=p_total[2]/ (float)num[2];
            m_average[2]=m_total[2]/ (float)num[2];
            break;
          case 3:
            num[3]+=1;
            diff_box[create.return_flag()][num[3]]=create.judge();
            if(max[3]<create.judge()){max[3] = create.judge();}
            if(min[3]>create.judge()){min[3] = create.judge();}
            total[3]= total[3]+create.judge();
            if(create.judge()>=0){
              p_num[3]+=1;
              p_total[3]=p_total[3]+create.judge();
            }else{
              m_num[3]+=1;
              m_total[3]=m_total[3]+create.judge();
            }
            average[3] = total[3] / (float)num[3];
            p_average[3]=p_total[3]/ (float)num[3];
            m_average[3]=m_total[3]/ (float)num[3];
            break;
          case 4:
            if(max[4]<create.judge()){max[4] = create.judge();}
            if(min[4]>create.judge()){min[4] = create.judge();}
            num[4]+=1;
            diff_box[create.return_flag()][num[4]]=create.judge();
            total[4]= total[4]+create.judge();

            if(create.judge()>=0){
              p_num[4]+=1;
              p_total[4]=p_total[4]+create.judge();
            }else{
              m_num[4]+=1;
              m_total[4]=m_total[4]+create.judge();
            }
            average[4] = total[4] / (float)num[4];
            p_average[4]=p_total[4]/ (float)num[4];
            m_average[4]=m_total[4]/ (float)num[4];
            break;
          case 5:
            if(max[5]<create.judge()){max[5] = create.judge();}
            if(min[5]>create.judge()){min[5] = create.judge();}
            num[5]+=1;
            diff_box[create.return_flag()][num[5]]=create.judge();
            total[5]= total[5]+create.judge();

            if(create.judge()>=0){
              p_num[5]+=1;
              p_total[5]=p_total[5]+create.judge();
            }else{
              m_num[5]+=1;
              m_total[5]=m_total[5]+create.judge();
            }
            average[5] = total[5] / (float)num[5];
            p_average[5]=p_total[5]/ (float)num[5];
            m_average[5]=m_total[5]/ (float)num[5];
            break;

          case 6:
            if(max[6]<create.judge()){max[6] = create.judge();}
            if(min[6]>create.judge()){min[6] = create.judge();}
            num[6]+=1;
            diff_box[create.return_flag()][num[6]]=create.judge();
            total[6]= total[6]+create.judge();

            if(create.judge()>=0){
              p_num[6]+=1;
              p_total[6]=p_total[6]+create.judge();

            }else{
              m_num[6]+=1;
              m_total[6]=m_total[6]+create.judge();
            }
            average[6] = total[6] / (float)num[6];
            p_average[6]=p_total[6]/ (float)num[6];
            m_average[6]=m_total[6]/ (float)num[6];
            break;

          case 7:
            if(max[7]<create.judge()){max[7] = create.judge();}
            if(min[7]>create.judge()){min[7] = create.judge();}
            num[7]+=1;
            diff_box[create.return_flag()][num[7]]=create.judge();
            total[7]= total[7]+create.judge();

            if(create.judge()>=0){
              p_num[7]+=1;
              p_total[7]=p_total[7]+create.judge();
            }else{
              m_num[7]+=1;
              m_total[7]=m_total[7]+create.judge();
            }
            average[7] = total[7] / (float)num[7];
            p_average[7]=p_total[7]/ (float)num[7];
            m_average[7]=m_total[7]/ (float)num[7];
            break;
          case 8:
            if(max[8]<create.judge()){max[8] = create.judge();}
            if(min[8]>create.judge()){min[8] = create.judge();}
            num[8]+=1;
            diff_box[create.return_flag()][num[8]]=create.judge();
            total[8]= total[8]+create.judge();

            if(create.judge()>=0){
              p_num[8]+=1;
              p_total[8]=p_total[9]+create.judge();
            }else{
              m_num[8]+=1;
              m_total[8]=m_total[8]+create.judge();
            }
            average[8] = total[8] / (float)num[8];
            p_average[8]=p_total[8]/ (float)num[8];
            m_average[8]=m_total[8]/ (float)num[8];
            break;
          case 9:
            if(max[9]<create.judge()){max[9] = create.judge();}
            if(min[9]>create.judge()){min[9] = create.judge();}
            num[9]+=1;
            diff_box[create.return_flag()][num[9]]=create.judge();
            total[9]= total[9]+create.judge();

            if(create.judge()>=0){
              p_num[9]+=1;
              p_total[9]=p_total[9]+create.judge();
            }else{
              m_num[9]+=1;
              m_total[9]=m_total[9]+create.judge();
            }
            average[9] = total[9] / (float)num[9];
            p_average[9]=p_total[9]/ (float)num[9];
            m_average[9]=m_total[9]/ (float)num[9];
            break;
          case 10:
            if(max[10]<create.judge()){max[10] = create.judge();}
            if(min[10]>create.judge()){min[10] = create.judge();}
            num[10]+=1;
            diff_box[create.return_flag()][num[10]]=create.judge();
            total[10]= total[10]+create.judge();
            if(create.judge()>=0){
              p_num[10]+=1;
              p_total[10]=p_total[10]+create.judge();
            }else{
              m_num[10]+=1;
              m_total[10]=m_total[10]+create.judge();
            }
            average[10] = total[10] / (float)num[10];
            p_average[10]=p_total[10]/ (float)num[10];
            m_average[10]=m_total[10]/ (float)num[10];
            break;
          case 0:
            break;
        }
      }
      //printf("%d\n",good_data);
    for(i=0;i<num[hit];i++){
      for(j=0;j<num[hit]-1;j++){
        if(diff_box[hit][num[hit]]>diff_box[hit][num[hit]+1]){
          tmp[hit]=diff_box[hit][num[hit]];
          diff_box[hit][num[hit]]=diff_box[hit][num[hit]+1];
          diff_box[hit][hit[num]+1]=tmp[hit];
        }
      }
    }


    if(key_num!=old_key_num){
      for(int res=0;res<11;res++){
        old_ave[res]=0.0;
      }
      cnt_flag=1;

    }

    if(cnt_flag != 0){
      first_cnt=1;
    }

    if(first_cnt==1){
      old_ave[1]=average[1];
      old_ave[2]=average[2];
      old_ave[3]=average[3];
      old_ave[4]=average[4];
      old_ave[5]=average[5];
      old_ave[6]=average[6];
      old_ave[7]=average[7];
      old_ave[8]=average[8];
      old_ave[9]=average[9];
      old_ave[10]=average[10];
      cnt_flag=0;
      first_cnt=0;
    }


    if(num[hit]%2==1){
      med[hit]=diff_box[hit][(num[hit]-1)/2];
    }else if (num[hit]%2==0){
      med[hit]= (diff_box[hit][(num[hit]/2)-1]+diff_box[hit][num[hit]/2])/2.0;
    }

    for(i=0;i<num[hit];i++){
      devi[hit] += (diff_box[hit][num[hit]]-average[hit])*(diff_box[hit][num[hit]]-average[hit]);
    }
    disp[hit]=devi[hit]/num[hit];
    std_devi[hit]=sqrtf(disp[hit]);
    create.med_create(med[hit],hit);

    for(int av=0;av<11;av++){
      if(fabsf(average[av]-old_ave[av])<=0.5){
        //average[hit]=old_ave[hit];
      }else{
        average[av]=old_ave[av];
        //average[hit]=average[hit];
      }
    }
    //old_ave[hit]=average[hit];
    old_ave[1]=average[1];
    old_ave[2]=average[2];
    old_ave[3]=average[3];
    old_ave[4]=average[4];
    old_ave[5]=average[5];
    old_ave[6]=average[6];
    old_ave[7]=average[7];
    old_ave[8]=average[8];
    old_ave[9]=average[9];
    old_ave[10]=average[10];



    hit_cnt[hit]++;
    old_hit=hit;
    old_key_num=key_num;
    printf("hit:%2d num:%2d ",hit,num[hit]);
    printf("hit%d:%6.2f\n",hit,average[hit]);
    //printf("ave1:%6.2f ave2:%6.2f ave3:%6.2f ave4:%6.2f ave5:%6.2f ave6:%6.2f ave7:%6.2f ave8:%6.2f ave9:%6.2f ave10:%6.2f \n ",average[1],average[2],average[3],average[4],average[5],average[6],average[7],average[8],average[9],average[10]);
    //printf("p_ave:%6.2f m_ave:%6.2f ",p_average[hit],m_average[hit]);
    //printf("max:%6.2f min:%6.2f ",max[hit],min[hit]);
    //printf("med:%6.2f 標準偏差:%6.3f\n",med[hit],std_devi[hit]);
  //  printf("hit_cnt:%d\n",hit_cnt[hit]);
    //printf("%6.3f\n",med[hit] );
    for(q=0;q<11;q++){
      tmp[q]=0.0;
      med[q]=0.0;
      num[q]=0;
      max[q]=0.0;
      min[q]=0.0;
      p_num[q]=0;
      m_num[q]=0;
      total[q]=0.0;
      p_total[q]=0.0;
      m_total[q]=0.0;
      average[q]=0.0;
      p_average[q]=0.0;
      m_average[q]=0.0;
      devi[q]=0.0;
      std_devi[q]=0.0;
      disp[q]=0.0;
      for(w=0;w<1080;w++){
        diff_box[q][w]=0.0;
      }
    }

  }

}

}
