//自動操縦の仕様が変わったので両チーム兼用になりました
//ゆっくり止まるようになりました
#include<Wire.h>

#define TEST_SONAR      0       //1だとON
#define TEST_AUTO     0       //0だとOFF

#define A_LAUNCH_CODE   0b01000000
#define O_LAUNCH_CODE   0b10000000
#define GO_STRAIGHT     0b00000001
#define BACK        0b00000010
#define LEFT        0b00000100
#define RIGHT         0b00001000
#define GO_45         0b00001001 //右ナナメ前
#define GO_135        0b00000101 //左ナナメ前
#define GO_225        0b00000110 //左ナナメ後ろ
#define GO_315        0b00001010 //右ナナメ後ろ
#define TURN_RIGHT      0b00001011 //旋回(時計回り)
#define TURN_LEFT     0b00000111 //旋回(反時計回り)
#define DISTURB_LEFT    0b00010000
#define DISTURB_RIGHT   0b00100000
#define AUTO_CONTROL    0b00001111
#define RC_CHECK_DATA   0b00000000  //未決定
#define FIRST_WALL      440  //超音波と壁の距離
#define SECOND_WALL     340
#define THIRD_WALL      240
#define FOURTH_WALL     140
#define FIFTH_WALL      1
#define FIRST_SPEED     60 //モータの速さの絶対値
#define SECOND_SPEED    50
#define THIRD_SPEED     40
#define FOURTH_SPEED    30
#define FIFTH_SPEED     15
#define TURN_SPEED      40
#define TARGET_DISTANCE_X 4775  //X軸のマシンの位置の目標値
#define TARGET_DISTANCE_Y 50    //Y軸のマシンの位置の目標値
#define RESET_ANGLE     60    //砲塔角度のリセット角度
#define MOTOR_COUNT     5     //モータ減速カウンター
//リモコン--------------------------------
unsigned long data_count = 100; //リモコンから通信が途切れても100周は回るやつ
//移動系--------------------------------
boolean TouchCheck = 0;   //多分使ってないマイクロスイッチ用
unsigned char motor_low_count = 0;    //モータがゆっくり止まるようにするフラグ的なもの
unsigned char motor_save_speed = 0;   //モータのスピードを入れる
unsigned char motor_save_vector = 0;  //モータの方向を入れる 前8,後2,左4,右6,右前9,左前7,左後1,右後3,時12,反24
//自動操縦------------------------------
unsigned char s_cho_data[4] = {0,0,0,0};     //計算時の仮置き場　省略できそう
unsigned int check_count;   //自動操縦時に連続で自動操縦しないようにするやつ

class slave{
private:  //外からはいじれない
  int8_t address;
  signed char s_send_data[1];
  unsigned char u_send_data[2];
  unsigned int received_int[2];
  unsigned char received_data[4]; //受信用4バイト

public:   //どこからでもいじれる
  slave(int addr){
    address = addr;
  }

  void s_send1(signed char s_data){ //符号付き1バイト データを入力するとsendされる
    s_send_data[0] = s_data;
    Wire.beginTransmission(address);
    Wire.write(s_send_data[0]);
    Wire.endTransmission(true);
  }

  void u_send1(unsigned char u_data){ //符号なし1バイト
    u_send_data[0] = u_data;
    Wire.beginTransmission(address);
    Wire.write(u_send_data[0]);
    Wire.endTransmission(true);
  }

  void u_send2(unsigned char u_data1, unsigned char u_data2){ //符号なし2バイト
    u_send_data[0] = u_data1;
    u_send_data[1] = u_data2;
    Wire.beginTransmission(address);
    Wire.write(u_send_data[0]);
    Wire.write(u_send_data[1]);
    Wire.endTransmission(true);
  }

  void receive1(int hoge = 1){
    Wire.requestFrom(address,hoge);
    received_data[0] = Wire.read();
  }

  void receive2(int hogehoge = 2){  //関数を呼び出せばその関数に値が入れられる privateなのでリードを使って読む
    Wire.requestFrom(address,hogehoge);
    received_data[0] = Wire.read();
    received_data[1] = Wire.read();  
  }

  void receive3(int hogehogehoge = 3){
    Wire.requestFrom(address,hogehogehoge);
    received_data[0] = Wire.read();
    received_data[1] = Wire.read();
    received_data[2] = Wire.read();
  }

  void receive4(int hogehogehogehoge = 4){
    Wire.requestFrom(address,hogehogehogehoge);
    received_data[0] = Wire.read();
    received_data[1] = Wire.read();
    received_data[2] = Wire.read();
    received_data[3] = Wire.read();
  }

  unsigned char read_char(int num){
    return received_data[num];
  }
  /*
  unsigned int read_int(int num1){
  return received_int[num1];
  }
  */
};

slave controler(0x10);
slave motor1(0x20);
slave motor2(0x21);
slave motor3(0x22);
slave motor4(0x23);
slave A_launch(0x30); //A砲塔発射   
slave A_control(0x31);  //砲塔回転
slave O_launch_A_reload(0x40);  //O砲塔発射、A補給　※送るデータによって変化
slave DualSonar(0x50);  //4バイトデータ
slave OnlySonar(0x51);
slave disturb(0x60);  //妨害

void setup(){
  Wire.begin();   //I2Cマスタ宣言
  Serial.begin(9600);
  Serial.println("START_ARAKI");
  //初期化処理未実装
  delay(100);
}

signed char speed_data(){         //速さと壁との距離はdefineで設定してください
  signed char motor_data = 0;    //計算時の仮置き場 モーターの速度の絶対値を入れる この関数の返り値
  unsigned char cho_data[4];     //計算時の仮置き場　省略できそう
  int left_data;
  int right_data;
  int only_data;
  if(TEST_SONAR == 1){  //超音波センサをテストするとき
    while(1){
      DualSonar.receive4(); //Y軸の超音波センサ
      cho_data[0] = DualSonar.read_char(0);
      cho_data[1] = DualSonar.read_char(1);
      cho_data[2] = DualSonar.read_char(2);
      cho_data[3] = DualSonar.read_char(3);
      left_data = cho_data[1] * 0x100 + cho_data[0];
      right_data = cho_data[3] * 0x100 + cho_data[2];
      if((1 > left_data) || (1 > right_data)){
        continue; //データが1より小さければもう一度データを取得し直す
      }
      else{ //そうでなければbreak
        break;
      }
    }
    while(1){
      OnlySonar.receive2(); //X軸の超音波センサ
      cho_data[0] = OnlySonar.read_char(0);
      cho_data[1] = OnlySonar.read_char(1);
      only_data = cho_data[1] * 0x100 + cho_data[0];
      if(1 > only_data){  //データが1より小さければもう一度データを取得し直す
        continue;
      }
      else{ //そうでなければbreak
        break;
      }
    }
    if((left_data >= FIRST_WALL) && (right_data >= FIRST_WALL) && (only_data >= FIRST_WALL)){ //400mm以上減速しない
      motor_data = FIRST_SPEED; //最高速
    }
    else if(((left_data >= SECOND_WALL) && (left_data < FIRST_WALL)) || ((right_data >= SECOND_WALL) && (right_data < FIRST_WALL)) || ((only_data >= SECOND_WALL) && (only_data < FIRST_WALL))){   //第1段階　300mm以上400mm未満
      motor_data = SECOND_SPEED;  //第二速
    }
    else if(((left_data >= THIRD_WALL) && (left_data < SECOND_WALL)) || ((right_data >= THIRD_WALL) && (right_data < SECOND_WALL)) || ((only_data >= THIRD_WALL) && (only_data < SECOND_WALL))){    //第2段階　200mm以上300mm未満
      motor_data = THIRD_SPEED; //第三速
    }
    else if(((left_data >= FOURTH_WALL) && (left_data < THIRD_WALL)) || ((right_data >= FOURTH_WALL) && (right_data < THIRD_WALL)) || ((only_data >= FOURTH_WALL) && (only_data < THIRD_WALL))){    //第3段階　100mm以上200mm未満
      motor_data = FOURTH_SPEED;  //第四速
    }
    else if(((left_data >= FIFTH_WALL) && (left_data < FOURTH_WALL)) || ((right_data >= FIFTH_WALL) && (right_data < FOURTH_WALL)) || ((only_data >= FIFTH_WALL) && (only_data < FOURTH_WALL))){    //第4段階　1mm以上100mm未満
      motor_data = FIFTH_SPEED; //第五速（死ぬほど遅い）
    }
    
    else{ //0以下

    }
  }
  else{ //超音波センサをテストしないとき
    motor_data = 60;
  }
  return motor_data;
}

void loop(){
  int count = 100;          //砲塔回転ループ用：カウンター
  int s_left_data = 0;        //mm単位で壁までの距離の値が入る　左側
  int s_right_data = 0;       //leftと同じ　右側
  int s_only_data = 0;    //mm単位で壁までの距離の値が入る　X軸
  int abs_data;       //leftからrightを引いた値の絶対値
  int average_data;     //leftとrightの平均値
  int angle_data;       //仮置き場
  signed char motor_speed = 0;    //モーターの速度の絶対値を入れる
  unsigned char rc_data[4];     //リモコンから受信したデータを入れる
  unsigned char rc_check = 0;   //リモコンのデータチェック
  unsigned char ct_angle = 0;     //砲塔回転ループ用：データ格納用
  boolean check_auto = 1;     //自動操縦ループ抜け用
  //------------------------------------------------------------------
  //受信処理 リモコンからデータがあれば、
  
  Wire.requestFrom(0x10,4);
  if(0 < Wire.available()){
    controler.receive4();
    rc_check = controler.read_char(3);  //リモコン受信チェック用データをrc_checkに入れる
  }
  else{
    if(rc_check == RC_CHECK_DATA){
      rc_data[0] = controler.read_char(0);
      rc_data[1] = controler.read_char(1);
      rc_data[2] = controler.read_char(2);
      rc_data[3] = controler.read_char(3);
      data_count = 0;
    }
    else{
      data_count++;
    }
  }

  /*
  controler.receive4();
  rc_check = controler.read_char(2);  //砲塔角度のデータをrc_checkに入れる
  if(0 < rc_check){ //砲塔角度のデータがある　= リモコンと通信できている
    controler.receive4();
    rc_data[0] = controler.read_char(0);
    rc_data[1] = controler.read_char(1);
    rc_data[2] = controler.read_char(2);
    rc_data[3] = controler.read_char(3);
    data_count = 0;
  }
  else{
    data_count++;
  }
  */
  /*
  if(0 < Serial.available()){
    rc_data[0] = Serial.read();
    data_count = 0;
  }
  else{
    data_count++;
  }
  */
  //-------------------------------------------------------------------
  if(data_count < 100){ //リモコンからデータが来れば走る
    switch(rc_data[0]){
    //発射処理
    case A_LAUNCH_CODE: //A砲塔発射
      Serial.println("A_launch");
      //マシンを停止
      motor1.s_send1(0);
      motor2.s_send1(0);
      motor3.s_send1(0);
      motor4.s_send1(0);
      A_control.u_send1(rc_data[2]);  //砲塔回転スタート
      while(count){ //砲塔回転完了まで待機 for文でないのはこだわり
        count--;  //countが0までデクリメント（100回繰り返す）
        A_control.receive1();
        ct_angle = A_control.read_char(0);
        if(ct_angle == rc_data[2]){
        break;
        }
        delay(30); //100回繰り返すので最大3秒待つ
      }
      A_launch.u_send2(127,rc_data[1]); //発射コマンドは0b01111111 = 127,発射飛距離は無修正木村データ
      delay(1000);
      //補給処理
      O_launch_A_reload.u_send1(31);  //補給コマンドは0b00011111 = 31
      //delay();
      break;

    case O_LAUNCH_CODE: //O砲塔発射
      Serial.println("O_launch");
      //マシンを停止
      motor1.s_send1(0);
      motor2.s_send1(0);
      motor3.s_send1(0);
      motor4.s_send1(0);
      O_launch_A_reload.u_send1(63);  //発射コマンドは0b00111111 = 63,発射飛距離は無し
      break;
    //移動処理　マイクロスイッチでの壁との接触判定無し　　自動操縦移動角度未定義
    case GO_STRAIGHT:
      Serial.println("GO_ST");
      motor_speed = speed_data();
      motor_save_speed = motor_speed;
      motor_save_vector = 8;
      motor_low_count = MOTOR_COUNT;
      motor1.s_send1(-motor_speed);
      motor2.s_send1(motor_speed);
      motor3.s_send1(motor_speed);
      motor4.s_send1(-motor_speed);
      break;

    case BACK:
      Serial.println("GO_BA");
      motor_speed = speed_data();
      motor_save_speed = motor_speed;
      motor_save_vector = 2;
      motor_low_count = MOTOR_COUNT;
      motor1.s_send1(motor_speed);
      motor2.s_send1(-motor_speed);
      motor3.s_send1(-motor_speed);
      motor4.s_send1(motor_speed);
      break;

    case LEFT:
      Serial.println("GO_LE");
      motor_speed = speed_data();
      motor_save_speed = motor_speed;
      motor_save_vector = 4;
      motor_low_count = MOTOR_COUNT;
      motor1.s_send1(-motor_speed);
      motor2.s_send1(-motor_speed);
      motor3.s_send1(motor_speed);
      motor4.s_send1(motor_speed);
      break;

    case RIGHT:
      Serial.println("GO_RI");
      motor_speed = speed_data();
      motor_save_speed = motor_speed;
      motor_save_vector = 6;
      motor_low_count = MOTOR_COUNT;
      motor1.s_send1(motor_speed);
      motor2.s_send1(motor_speed);
      motor3.s_send1(-motor_speed);
      motor4.s_send1(-motor_speed);
      break;

    case GO_45: //右ナナメ前
      Serial.println("GO_45");
      motor_speed = speed_data();
      motor_save_speed = motor_speed;
      motor_save_vector = 9;
      motor_low_count = MOTOR_COUNT;
      motor1.s_send1(0);
      motor2.s_send1(motor_speed);
      motor3.s_send1(0);
      motor4.s_send1(-motor_speed);
      break;

    case GO_135: //左ナナメ前
      Serial.println("GO_135");
      motor_speed = speed_data();
      motor_save_speed = motor_speed;
      motor_save_vector = 7;
      motor_low_count = MOTOR_COUNT;
      motor1.s_send1(-motor_speed);
      motor2.s_send1(0);
      motor3.s_send1(motor_speed);
      motor4.s_send1(0);
      break;

    case GO_225: //左ナナメ後ろ
      Serial.println("GO_225");
      motor_speed = speed_data();
      motor_save_speed = motor_speed;
      motor_save_vector = 1;
      motor_low_count = MOTOR_COUNT;
      motor1.s_send1(0);
      motor2.s_send1(-motor_speed);
      motor3.s_send1(0);
      motor4.s_send1(motor_speed);
      break;

    case GO_315: //右ナナメ後ろ
      Serial.println("GO_315");
      motor_speed = speed_data();
      motor_save_speed = motor_speed;
      motor_save_vector = 3;
      motor_low_count = MOTOR_COUNT;
      motor1.s_send1(motor_speed);
      motor2.s_send1(0);
      motor3.s_send1(-motor_speed);
      motor4.s_send1(0);
      break;

    case TURN_LEFT: //旋回(反時計回り)
      Serial.println("TU_LE");
      motor_speed = speed_data();
      motor_save_speed = motor_speed;
      motor_save_vector = 12;
      motor_low_count = MOTOR_COUNT;
      motor1.s_send1(-motor_speed);
      motor2.s_send1(-motor_speed);
      motor3.s_send1(-motor_speed);
      motor4.s_send1(-motor_speed);
      break;

    case TURN_RIGHT: //旋回(時計回り)
      Serial.println("TU_RI");
      motor_speed = speed_data();
      motor_save_speed = motor_speed;
      motor_save_vector = 24;
      motor_low_count = MOTOR_COUNT;
      motor1.s_send1(motor_speed);
      motor2.s_send1(motor_speed);
      motor3.s_send1(motor_speed);
      motor4.s_send1(motor_speed);
      break;

    case DISTURB_LEFT:
      Serial.println("DI_LE");
      disturb.s_send1(-127);
      motor1.s_send1(0);
      motor2.s_send1(0);
      motor3.s_send1(0);
      motor4.s_send1(0);
      break;

    case DISTURB_RIGHT:
      Serial.println("DI_RI");
      disturb.s_send1(127);
      motor1.s_send1(0);
      motor2.s_send1(0);
      motor3.s_send1(0);
      motor4.s_send1(0);
      break;

    case AUTO_CONTROL: //赤チーム時 自動操縦の仕様変更により、位置合わせのみになった
      if(TEST_AUTO){
        if(check_auto){
          Serial.println("START_AUTO_ARAKI");
          motor1.s_send1(0);
          motor2.s_send1(0);
          motor3.s_send1(0);
          motor4.s_send1(0);
          while(1){ //まず自陣ポールの方のフェンスを基準にマシンを平行にする
            controler.receive4();
            rc_data[0] = controler.read_char(0);
            if(rc_data[0] != AUTO_CONTROL){ //コントローラーから自動操縦以外の通信があったら中断する
              Serial.println("STOP_AUTO");
              check_auto = 0;
              break;
            }
            else{
              while(1){ //超音波センサーからY軸の値を取得
                DualSonar.receive4();
                s_cho_data[0] = DualSonar.read_char(0);
                s_cho_data[1] = DualSonar.read_char(1);
                s_cho_data[2] = DualSonar.read_char(2);
                s_cho_data[3] = DualSonar.read_char(3);
                s_left_data = s_cho_data[1] * 0x100 + s_cho_data[0];
                s_right_data = s_cho_data[3] * 0x100 + s_cho_data[2];
                if((1 > s_left_data) || (1 > s_right_data)){
                  continue; //データが1より小さければもう一度データを取得し直す
                }
                else{ //そうでなければbreak
                  break;
                }
              }
              abs_data = abs(s_left_data - s_right_data);
              if(abs_data > 10){  //+-10ミリで平行ならbreakして次の処理へ
                motor1.s_send1(0);
                motor2.s_send1(0);
                motor3.s_send1(0);
                motor4.s_send1(0);
                break;
              }
              else{
                if(s_right_data < s_left_data){   //leftの値の方が大きい(時計回り)
                  motor_speed = 15;
                  motor1.s_send1(motor_speed);
                  motor2.s_send1(motor_speed);
                  motor3.s_send1(motor_speed);
                  motor4.s_send1(motor_speed);
                }
                else if(s_left_data < s_right_data){  //rightの値の方が大きい(反時計回り)
                  motor_speed = 15;
                  motor1.s_send1(-motor_speed);
                  motor2.s_send1(-motor_speed);
                  motor3.s_send1(-motor_speed);
                  motor4.s_send1(-motor_speed);
                }
              }
            }
          }//whileフェンス平行
          while(check_auto){  //次にOnlySonarを使ってX軸の位置合わせをする
            controler.receive4();
            rc_data[0] = controler.read_char(0);
            if(rc_data[0] != AUTO_CONTROL){ //コントローラーから自動操縦以外の通信があったら中断する
              Serial.println("STOP_AUTO");
              check_auto = 0;
              break;
            }
            else{
              while(1){
                OnlySonar.receive2();
                s_cho_data[0] = OnlySonar.read_char(0);
                s_cho_data[1] = OnlySonar.read_char(1);
                s_only_data = s_cho_data[1] * 0x100 + s_cho_data[0];
                if(1 > s_only_data){
                  continue; //データが1より小さければもう一度データを取得し直す
                }
                else{ //そうでなければbreak
                  break;
                }
              }
              abs_data = abs(s_only_data - TARGET_DISTANCE_X);  //絶対値（absolute）を取る
              if(abs_data > 5){ //+-5ミリで平行ならbreakして次の処理へ
                motor1.s_send1(0);
                motor2.s_send1(0);
                motor3.s_send1(0);
                motor4.s_send1(0);
                break;
              }
              else{
                if(s_only_data < TARGET_DISTANCE_X){  //左に
                  motor_speed = 30;
                  motor1.s_send1(-motor_speed);
                  motor2.s_send1(-motor_speed);
                  motor3.s_send1(motor_speed);
                  motor4.s_send1(motor_speed);
                }
                else if(TARGET_DISTANCE_X < s_only_data){ //右に
                  motor_speed = 30;
                  motor1.s_send1(motor_speed);
                  motor2.s_send1(motor_speed);
                  motor3.s_send1(-motor_speed);
                  motor4.s_send1(-motor_speed);
                }
              }
            }
          }
          while(check_auto){
            controler.receive4();
            rc_data[0] = controler.read_char(0);
            if(rc_data[0] != AUTO_CONTROL){ //コントローラーから自動操縦以外の通信があったら中断する
              Serial.println("STOP_AUTO");
              check_auto = 0;
              break;
            }
            else{ //自動操縦している
              while(1){ //超音波センサの値を取得
                DualSonar.receive4();
                s_cho_data[0] = DualSonar.read_char(0);
                s_cho_data[1] = DualSonar.read_char(1);
                s_cho_data[2] = DualSonar.read_char(2);
                s_cho_data[3] = DualSonar.read_char(3);
                s_left_data = s_cho_data[1] * 0x100 + s_cho_data[0];
                s_right_data = s_cho_data[3] * 0x100 + s_cho_data[2];
                if((1 > s_left_data) || (1 > s_right_data)){
                  continue;
                }
                else{
                  break;
                }
              }
              average_data = (s_left_data + s_right_data) * 0.5;
              abs_data = abs(average_data - TARGET_DISTANCE_Y);
              if(abs_data > 10){
                check_auto = 0;   //連続で自動操縦しないように0を代入
                break;
              }
              else{
                if(average_data > TARGET_DISTANCE_Y){   //前に
                  motor_speed = 30;
                  motor1.s_send1(-motor_speed);
                  motor2.s_send1(motor_speed);
                  motor3.s_send1(motor_speed);
                  motor4.s_send1(-motor_speed);
                }
                else if(TARGET_DISTANCE_Y < average_data){  //後ろに
                  motor_speed = 30;
                  motor1.s_send1(motor_speed);
                  motor2.s_send1(-motor_speed);
                  motor3.s_send1(-motor_speed);
                  motor4.s_send1(motor_speed);
                }
              }
            }
          }
        }//if(check_auto)
        else{ //自動操縦が正常 or 異常終了した
          if(check_count < 10000){
            ++check_count;
          }
          else{ //check_countが10000を超えると再度、自動操縦が出来るようになる
            check_count = 0;
            check_auto = 1;
          }
        }
        motor1.s_send1(0);
        motor2.s_send1(0);
        motor3.s_send1(0);
        motor4.s_send1(0);
      }
      else{ //TEST_AUTOに対してのelse
      }
      break;

    default:
      if (1 <= rc_data[3]) {  //砲塔リセットのデータがある
        Serial.println("RESET_ANGLE");
        A_control.u_send1(RESET_ANGLE);  //砲塔リセットデータを送る（値はdefine）
        //なんか必要になったらdelayしよう（多分回しっぱで大丈夫と思う）
        //delay(1800); //砲塔が回転するまで待機
      }
      else {  //砲塔リセットのデータ無し
        if (0 < rc_data [2]) {  //砲塔角度のデータあり
          Serial.print("ANGLE ");
          angle_data = rc_data[2] * 0.47058823529;                        //rc_data[2] * 120 / 255
          unsigned char angle_true_data = (unsigned char)(int)angle_data; //キャストして少数切り捨てするンゴｗｗｗｗｗｗｗｗｗｗｗｗ
          Serial.println(angle_true_data);
          A_control.u_send1(angle_true_data);  //砲塔回転
        }
        else { //砲塔角度のデータ無し
        }
      }
      A_launch.u_send2(0, rc_data[1]);  //発射威力送信
      if(0 < motor_low_count){  //モータをゆっくり止める系の処理、移動コマンドの処理時に保存したデータを元に減速していく
        motor_low_count--;
        motor_save_speed = motor_save_speed >> 1; //1ビット右シフト = 1/2
        switch(motor_save_vector){
        case 8://前方向
          motor1.s_send1(-motor_save_speed);
          motor2.s_send1(motor_save_speed);
          motor3.s_send1(motor_save_speed);
          motor4.s_send1(-motor_save_speed);
          break;
        case 2://後ろ方向
          motor1.s_send1(motor_save_speed);
          motor2.s_send1(-motor_save_speed);
          motor3.s_send1(-motor_save_speed);
          motor4.s_send1(motor_save_speed);
          break;
        case 4://左方向
          motor1.s_send1(-motor_save_speed);
          motor2.s_send1(-motor_save_speed);
          motor3.s_send1(motor_save_speed);
          motor4.s_send1(motor_save_speed);
          break;
        case 6://右方向
          motor1.s_send1(motor_save_speed);
          motor2.s_send1(motor_save_speed);
          motor3.s_send1(-motor_save_speed);
          motor4.s_send1(-motor_save_speed);
          break;
        case 9://45方向
          motor1.s_send1(0);
          motor2.s_send1(motor_save_speed);
          motor3.s_send1(0);
          motor4.s_send1(-motor_save_speed);
          break;
        case 7://135方向
          motor1.s_send1(-motor_save_speed);
          motor2.s_send1(0);
          motor3.s_send1(motor_save_speed);
          motor4.s_send1(0);
          break;
        case 1://225方向
          motor1.s_send1(0);
          motor2.s_send1(-motor_save_speed);
          motor3.s_send1(0);
          motor4.s_send1(motor_save_speed);
          break;
        case 3://315方向
          motor1.s_send1(motor_save_speed);
          motor2.s_send1(0);
          motor3.s_send1(-motor_save_speed);
          motor4.s_send1(0);
          break;
        case 12://時計回り
          motor1.s_send1(motor_save_speed);
          motor2.s_send1(motor_save_speed);
          motor3.s_send1(motor_save_speed);
          motor4.s_send1(motor_save_speed);
          break;
        case 24://反時計回り
          motor1.s_send1(-motor_save_speed);
          motor2.s_send1(-motor_save_speed);
          motor3.s_send1(-motor_save_speed);
          motor4.s_send1(-motor_save_speed);
          break;
        default: //(defaultに値が来ることは想定して)ないです。
          break;
        }//switch(motor_save_vector)
      }
      else{ //停止
        motor1.s_send1(0);
        motor2.s_send1(0);
        motor3.s_send1(0);
        motor4.s_send1(0);
      }
      disturb.s_send1(0);
      break;
    }//switch
  }//if(data_count)
  else {  //データカウントが100以上(リモコンからデータが送られなくて連続100回目の時の処理)
    rc_data[0] = 0; //データをリセット(もし、オーバーフローしてif(data_count)の処理に入っても大丈夫なように)
      rc_data[1] = 0;
      rc_data[2] = 0;
      rc_data[3] = 0;
    motor1.s_send1(0); //モーターを停止
    motor2.s_send1(0);
    motor3.s_send1(0);
    motor4.s_send1(0);
    disturb.s_send1(0); //妨害装置を停止
  }//else(data_count)
}//loop
