
int distance = 0;
int radius = 0;
int xcoord=0;
double angle=0;

void setup(){
  
}

void loop(){
  angle=pixeltoangle(radius,xcoord);
}

  double pixeltoangle(int radius,int  xpixel) {
  distance = -1.52*radius + 137;
  int xpixeladjusted = xpixel - 320;
  float relativePixel = xpixeladjusted/320;
  double Angle = relativePixel*32.6;
  return Angle;
}
