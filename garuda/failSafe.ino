void failSafe(){
  unsigned minVal=800;
  unsigned maxVal=2200;

  int check1=0;
  int check2=0;
  int check3=0;
  int check4=0;
  int check5=0;
  if (ch1_value > maxVal || ch1_value < minVal) check1 = 1;
  if (ch2_value > maxVal || ch2_value < minVal) check2 = 1;
  if (ch3_value > maxVal || ch3_value < minVal) check3 = 1;
  if (ch4_value > maxVal || ch4_value < minVal) check4 = 1;
  if (ch5_value > maxVal || ch5_value < minVal) check5 = 1;

  if ((check1 + check2 + check3 + check4 + check5 ) > 0) {
    ch1_value = channel_1_fs;
    ch2_value = channel_2_fs;
    ch3_value = channel_3_fs;
    ch4_value = channel_4_fs;
    ch5_value = channel_5_fs;
}
}
