// 17/03/08 R3+【SD_TIMEシールド】+NEO_6M半田付け基盤 XG01
//   ＳＤカード　初期化失敗時　0.5秒のLED点滅で警告　動作ストップ
// 変更点：ファイル名の取得をＧＰＳ測位データからＲＴＣに変更
// 02:上記変更に伴いチェックデータ時分秒に
// 03:タイムスタンプ　記録開始時のデータの位置設定
// NEO-6M:  (TX pin ->  Arduino RX) -> Arduino Digital 4
//          (RX pin ->  Arduino TX) -> Arduino Digital 5

// SD:      CS pin    -> Arduino Digital 10
//          SCK pin   -> Arduino Digital 13
//          MOSI pin  -> Arduino Digital 11
//          MISO pin  -> Arduino Digital 12
//          VCC pin   -> +5
//          GND       -> GND
//  LED       -> 470Ω     -> Arduino Digital 3
//  SW (p-up) -> 10kΩ     -> Arduino Digital 2
//備考　保存データがGPRMC + GPGGAであればchar nmea[140];を[90];位に変更できそう
//動作メモリ確保の為　できるだけ関数使わずべた書き
/*
最大32256バイトのフラッシュメモリのうち、スケッチが21292バイト（66%）を使っています。
最大2048バイトのRAMのうち、グローバル変数が1723バイト（84%）を使っていて、ローカル変数で325バイト使うことができます。
スケッチが使用できるメモリが少なくなっています。動作が不安定になる可能性があります。
最大32256バイトのフラッシュメモリのうち、スケッチが21226バイト（65%）を使っています。
最大2048バイトのRAMのうち、グローバル変数が1717バイト（83%）を使っていて、ローカル変数で331バイト使うことができます。
最大32256バイトのフラッシュメモリのうち、スケッチが21336バイト（66%）を使っています。
最大2048バイトのRAMのうち、グローバル変数が1717バイト（83%）を使っていて、ローカル変数で331バイト使うことができます。
*/
#include <SoftwareSerial.h>
#include <Wire.h>
#include <TimeLib.h>
#include <Adafruit_BMP085.h>
#include <SD.h>
#define FLAG_FILE    1 //0000 0001
#define FLAG_LED3    2 //0000 0010
#define FLAG_SW1     4 //0000 0100
#define FLAG_SW2     8 //0000 1000
#define FLAG_A1     16 //0001 0000
#define FLAG_A2     32 //0010 0000
#define FLAG_break  64 //0100 0000
#define FLAG_box   128 //1000 0000
//■■■■下記必須変数
SoftwareSerial g_gps(4, 5); // RX, TX
const int chipSelect = 10; //SDカード　シールド CS  10
File myFile;//ファイルポインタ
char filename[13];
//int swin1, swin2; //スイッチ　ｏｎ－ｏｆｆでファイル制御
//int files_sw1 = 0;

int flag=0;// ファイル　ｌｅｄ　スイッチ　兼用　breakflag = 0; //while文脱出フラグ

int comma[30]; // カンマカウント
int ck = 0; //
int k;
//int LED3_SW;       //
char nmea[101];    //メインテーブル
//■■■■
tmElements_t tm;

//■■■■BMP085追加
Adafruit_BMP085 bmp;

//■■■■時間
unsigned long LED3_TIME;//LED動作カウンタ　動作含める
unsigned long LED3_INVAL = 500;//0.5秒おきに送信

unsigned long RECON_TIME;//FILE動作カウンタ　動作含める
unsigned long GET_TIME;//FILE動作カウンタ　動作含める
unsigned long FILE_TIME;//FILE動作カウンタ　動作含める
unsigned long FILE_INVAL = 60000;//60000=1分　120000=2分　180000=3分
//■■■■予備変数
float wpa = 0; //　Ph
float wdo = 0; //  ℃

//■■■■■■■■■■■■■■■■■■■■
//    setup()
//■■■■■■■■■■■■■■■■■■■■
void setup()
{
  Serial.begin(115200);   //ＵＳＢ速度
  g_gps.begin(9600);      //ＧＰＳユニット
  pinMode(2, INPUT) ;     //スイッチに接続ピンをデジタル入力に設定
  pinMode(3, OUTPUT);     // LED用
  digitalWrite(3, LOW);   //■■■■動作時ｌｅｄ　ｏｆｆ

  //■■■■ファイル処理
 // files_sw1 = 0;  //defでファイルスイッチＯＮ

  //■■■■バージョン表記
  Serial.println("GPS_XG01_test_03-2-4.ino ver:2018-09-17 RTC");
  Serial.println("compile time 13:12");
  //■■■■ＳＤカード
  if (!SD.begin(chipSelect)) { //CSピン番号を指定しない場合のデフォルトはSSピン(Unoの場合は10番ピン)です
    // カードの初期化に失敗したか、またはＳＤが入っていない
    Serial.println("SDcard init NG");
    while (1) {
      digitalWrite(3, HIGH);   // off時 LED on
      delay (500);
      digitalWrite(3, LOW);   // ｌｅｄ　ｏｆｆ
      delay (500);
    }
    //return;
  }
  else {
    Serial.println("SDcard init OK");
  }
  //■■■■ＳＤカード end

  for (k = 0; k < 30; k++) {
    comma[k] = 0;
  }
  //■■■■BMP180
  if (!bmp.begin()) {
    Serial.println("BMP180 sensor, check wiring!");
    while (1) {
      digitalWrite(3, HIGH);   // off時 LED on
      delay (500);
      digitalWrite(3, LOW);   // ｌｅｄ　ｏｆｆ
      delay (500);
    }
  }
  //swin2 = swin1 = 0;

}
//■■■■■■■■■■■■■■■■■■■■
//    LOOP
//■■■■■■■■■■■■■■■■■■■■
void loop()
{
  //////////////////////////////////////////////////
  //    入力
  //////////////////////////////////////////////////
//int swin1, swin2; //スイッチ　ｏｎ－ｏｆｆでファイル制御
//int files_sw1 = 0;
  if(flag & FLAG_SW1){flag |= FLAG_SW2;}else{flag &=~FLAG_SW2;}//状態保存   swin2 = swin1;
  if (digitalRead(2) == LOW ) { //スイッチの状態を調べる　プルアップ抵抗 low時キーＯＮ
    flag |=FLAG_SW1 ;//sw1 に1
  }
  else {
    flag &=~FLAG_SW1 ;//sw1 に0
  }
  //////////////////////////////////////////////////
  //    判定
  //////////////////////////////////////////////////
//  if (swin1 == 1 && swin2 == 0) { //■立ち上げ時 ON
  if ((flag & FLAG_SW1) && !(flag & FLAG_SW2))  { //■立ち上げ時 ON
    Serial.println("---  FILE record sw ON    ---");
    GET_TIME = LED3_TIME = millis();     //■■■■ 時間ゲット
    FILE_TIME = GET_TIME + FILE_INVAL;  //インターバルタイマ
    LED3_TIME = LED3_TIME + LED3_INVAL; //

    flag |=FLAG_LED3; digitalWrite(3, HIGH); //  LED on
    
   // int breakflag = 0; //
    flag &=~FLAG_break ;// while文脱出フラグ FLAG_break に0
   
    while (1) { //●
      led3_sw_flas();//点滅割り込み
      one_line_read();//1行読んで
      if (nmea[3] == 'R' && nmea[4] == 'M' && nmea[5] == 'C') { //判定 RMCだったら
            //Serial.println("RRRR");
        k = gps_nmea_rcm();                          //データチェック
        if (k == 0) {                                //エラー無しデータまで読み出し
           //Serial.println("MMMM");
          rmc_dateTime();                           //RMCからtmへ時刻転送
          UTC_DateTimeConv(9);                      //tmからJST(+9)処理1～+23まで
          SdFile::dateTimeCallback(&dateTime);    // 日付と時刻を返す関数を登録
          filecop();                                //ファイル名生成+ファイルop
          myFile.println(nmea);                     //最初の1行書き込み　他のデータは省略　不関数でメモリ対策
          //Serial.println("CCC");
          //breakflag = 1;                          //終了フラグ
          flag |=FLAG_break;                        //終了フラグ
          break;
        }
        if (flag & FLAG_break) break;
      }
      if (flag & FLAG_break) break;

      if (digitalRead(2) == HIGH ) { //スイッチの状態を調べる HI時キーOFF ＯＮ=>OFF時測位中キャンセル
         flag &=~FLAG_SW1; digitalWrite(3, LOW);
        break;
      }
    }//●while終了
    RECON_TIME = millis();     //■■■■ 時間ゲット
    Serial.println("---  FILE record start    ---");
  }//■立ち上げ時 ON 終了
  ///////////////////////////////////////////
  if (!(flag & FLAG_SW1) && (flag & FLAG_SW2)) {//■立下りｏｆｆ
    Serial.println("--- FILE  end  ---");
    fileccl();//ファイルクローズ
  }//■

  //////////////////////////////////////////////////
  //    メイン
  //////////////////////////////////////////////////
  GET_TIME = millis();//ｓｗ-ｏｎ時間保存
  one_line_read();// 主処理
}
//■■■■■■■■■■■■■■■■■■■■
//　　　　　　ファイル名生成
//■■■■■■■■■■■■■■■■■■■■
void filenamemake_JST()
{
  //GPSデータＵＴＣなので日本時間　UTC_DateTimeConv()直後に呼ぶ事
  String cy =  String((tm.Year + 1970) - 2000);
  //Serial.println(cy);
  String cm =  String(tm.Month);
  //Serial.println(cm);
  String cd =  String(tm.Day);
  //Serial.println(cd);
  String ch =  String(tm.Hour);
  //Serial.println(ch);
  filename[0] = '2' ;
  filename[1] = '0' ;
  filename[2] = cy[0];//年
  filename[3] = cy[1];
  //月
  if (tm.Month >= 10) {
    filename[4] = cm[0];
    filename[5] = cm[1];
  }
  else {
    filename[4] = '0';
    filename[5] = cm[0];
  }
  //日
  if (tm.Day >= 10) {
    filename[6] = cd[0];
    filename[7] = cd[1];
  }
  else {
    filename[6] = '0';
    filename[7] = cd[0];
  }
  filename[8] = '.' ;
  filename[9] = 'l' ;
  filename[10] = 'o' ;
  filename[11] = 'g' ;
  filename[12] = '\0' ;

}

//■■■■■■■■■■■■■■■■■■■■
//    NMEAデータ　チェック1　データなし時
//■■■■■■■■■■■■■■■■■■■■
int NMEA_data_chk1(int a)
{
  int w;
  w = a + 1;

  if (nmea[w] == ',') {
    return 1;//データ無し
  }

  if (nmea[w] == '9') {
    return 1;//
  }
  else {
    return 0;
  }

}
//■■■■■■■■■■■■■■■■■■■■
//     GPS RMC 日付データ　チェック
//■■■■■■■■■■■■■■■■■■■■
int gps_nmea_rcm() {
  int w;
  int p = 0;
  ////日月年UTC　, [8]-[9]
  w = comma[0];
  if (NMEA_data_chk1(w) == 1) {
    //Serial.println("ERR1");
    p++;
  }//エラーがあれば+1
  w = comma[8];
  if (NMEA_data_chk1(w) == 1) {
    //Serial.println("ERR2");
    p++;
  }//エラーがあれば

  return p;

}
//■■■■■■■■■■■■■■■■■■■■
//      ファイルオープン処理
//■■■■■■■■■■■■■■■■■■■■
void filecop()
{
  flag |=FLAG_FILE ;//ファイルスイッチＯＮ 1
//  files_sw1 = 1;//ファイルスイッチＯＮ
  filenamemake_JST();//ファイル名生成

  myFile = SD.open(filename, FILE_WRITE);
  delay (20);
  digitalWrite(3, HIGH);   //  LED on
  Serial.println("File.open");
}
//■■■■■■■■■■■■■■■■■■■■
//      ファイルクローズ処理
//■■■■■■■■■■■■■■■■■■■■
void fileccl()
{
  //files_sw1 = 0;
  flag &=~FLAG_FILE;// sw 0
  //ファイルクローズ処理
  myFile.close();
  delay (1000);
  digitalWrite(3, LOW);   //  LED off　
  Serial.println("File.close");
}

//■■■■■■■■■■■■■■■■■■■■
//      スイッチ点滅
//■■■■■■■■■■■■■■■■■■■■
void  led3_sw_flas()
{
  if (millis() > LED3_TIME) {
    if (flag & FLAG_LED3) {
       flag &=~FLAG_LED3 ; digitalWrite(3, LOW); // offに
    }
    else {
       flag |=FLAG_LED3 ; digitalWrite(3, HIGH); // onに
    }
    LED3_TIME = millis();     //■■■■ 時間ゲット
    LED3_TIME = LED3_TIME + LED3_INVAL;
  }
}

//■■■■■■■■■■■■■■■■■■■■
//     1行読み込み
//■■■■■■■■■■■■■■■■■■■■
int one_line_read()
{
 // int breakflag = 0; // #define FLAG_box  
  int v = 0;        //行カウンタリセット
  int ck = 0; //comma[k]位置最大　リセット
  unsigned long  s;
  flag &=~FLAG_box ;//breakフラグ　0クリア
  while (1) { //●
    //以下 GPS　データリード　(1文字)関数べた書き■■■■
    while (1) {
      nmea[v] = g_gps.read();//■gpsデータ読み出し
      if ( -1 == nmea[v] || 0x0a == nmea[v])continue;  //-1の時は読み飛ばし
      //if( -1 == nmea[v] )continue;   //-1の時は読み飛ばし
      else {
        //Serial.print(nmea[v]);
        break;
      }
    }
    //GPS　データリード　(1文字)関数べた書き終了■■■■
    if (',' == nmea[v]) {
      comma[ck] = v;  //カンマカウント
      ck++;
    }
    //■■■■■■■■■■■■■■■■■■■■
    //
    //■■■■■■■■■■■■■■■■■■■■
    if ( 0x0d == nmea[v]) //改行
    {
      nmea[v] = '\0'; //端末処理

    //  if ( nmea[3] == 'R' || nmea[4] == 'G') {
      if ( nmea[3] == 'R') {
        wpa = bmp.readPressure() / (float)100; //　Ph
        wdo = bmp.readTemperature(); //  ℃
      }
      if ((flag & FLAG_FILE) && ( nmea[3] == 'R')) { //sw1で書き込み   GPRMC + GPGGAの書き込み

        // GET_TIME = GET_TIME -  RECON_TIME;

        myFile.print(nmea);//■■■■■ 書き込み
        if (nmea[3] == 'R') {
          myFile.print(',');
          myFile.print(wpa);//Pa
          myFile.print(',');
          myFile.print(wdo);//℃
        }
        myFile.println();//端末改行
        Serial.println(nmea); //■■■■■ 表示
      }
      else {
        Serial.print(nmea); //■■■■■ 表示
        if (nmea[3] == 'R' || nmea[4] == 'G') {
          Serial.print(',');
          Serial.print(wpa);//Pa
          Serial.print(',');
          Serial.print(wdo);//℃
        }
        Serial.println();
      }
      v = 0;  //行カウンタリセット
      ck = 0; //comma[k]位置最大　リセット
       flag |=FLAG_box ; //ループ終了フラグ　on　1
     // breakflag = 1; //ループ終了フラグ
    }//改行処理終了 TRUE
    else {
      v++; if (v >= 99)v = 99;        ////■■■■■■■■■■■■■■■■■■■■念のため
    }//改行処理終了 FALSE
    //////////////////////////////////////////
    if (flag & FLAG_box ) break; //ループ終了
  }
}
//■■■■■■■■■■■■■■■■■■■■
//   ＳＤカードタイムスタンプ
//■■■■■■■■■■■■■■■■■■■■
void dateTime(uint16_t* date, uint16_t* time)
{
  // (RTC.read(tm));
  uint16_t year = tm.Year + 1970;// + 1970 必須
  uint8_t month = tm.Month;
  // GPSやRTCから日付と時間を取得
  // FAT_DATEマクロでフィールドを埋めて日付を返す
  *date = FAT_DATE(year, month, tm.Day);
  // FAT_TIMEマクロでフィールドを埋めて時間を返す
  *time = FAT_TIME(tm.Hour, tm.Minute, tm.Second);
}
//■■■■■■■■■■■■■■■■■■■■
//   RMCから年月日時分秒抜き出し  2017年時　経過年数47
//■■■■■■■■■■■■■■■■■■■■
void rmc_dateTime()
{
  int w;
  char s[5];
  w = comma[8]; w++; //(日月年)指定位置カンマから数字先頭へ

  //■■年  (20)17-1970  経過年数47
  s[0] = nmea[w + 4];
  s[1] = nmea[w + 5];
  s[2] = '\0';
  tm.Year = atoi(s) + 30; //変換時+30で経過年数へ
  //■■月
  s[0] = nmea[w + 2];
  s[1] = nmea[w + 3];
  s[2] = '\0';
  tm.Month = atoi(s);
  //■■日
  s[0] = nmea[w + 0];
  s[1] = nmea[w + 1];
  s[2] = '\0';
  tm.Day = atoi(s);

  w = comma[0]; w++; //(時分秒)指定位置カンマから数字先頭へ
  //■■時
  s[0] = nmea[w + 0];
  s[1] = nmea[w + 1];
  s[2] = '\0';
  tm.Hour = atoi(s);
  //■■分
  s[0] = nmea[w + 2];
  s[1] = nmea[w + 3];
  s[2] = '\0';
  tm.Minute = atoi(s);
  //■■秒
  s[0] = nmea[w + 4];
  s[1] = nmea[w + 5];
  s[2] = '\0';
  tm.Second = atoi(s);//秒　

}
//■■■■■■■■■■■■■■■■■■■■
//  世界時から日本時間へ (とりあえず+時間のみ)　UTC_DateTimeConv(9); //UTCからJSTへ
//■■■■■■■■■■■■■■■■■■■■
int UTC_DateTimeConv(int s)
{
  int ndays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  int w = 0;
  if (s > 23 || s < -23)return 1; //エラー
  if (s > 0) {
    tm.Hour = tm.Hour + s ;//時間加算
    if (tm.Hour >= 24) {
      tm.Day++;
      tm.Hour = tm.Hour % 24;
    }
    else {
      return 0;         //日付変わらないならそのまま
    }

    //tm.Day　最大値チェック

    if (tm.Month == 2 && is_leap_year( tm.Year + 1970 ) == 1) { //2月ならうるう年チェック
      //  Serial.println("ok");
      if (tm.Day > 29) {  //うるう年の2月
        tm.Day = 1;
        tm.Month = 3;
        return 0;         //でのまま終了
      }
    }
    else {
      if (tm.Day >= ndays[tm.Month - 1]) {
        tm.Day = 1;
        tm.Month = tm.Month + 1;
      }
    }
    //////////////////////////////
    if (tm.Month > 12) {        //12月超えたら　年+1
      tm.Year = tm.Year + 1;
      tm.Month = 1;
    }
  }
}
//■■■■■■■■■■■■■■■■■■■■
//  うるう年判定　(1:うるう年 0:平年)
//■■■■■■■■■■■■■■■■■■■■
int is_leap_year(int year)
{
  if (year % 400 == 0) return 1;
  else if (year % 100 == 0) return 0;
  else if (year % 4 == 0) return 1;
  else return 0;
}


