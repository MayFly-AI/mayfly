#include "medianfilter.h"

#define MFSORT2(a,b) if(a>b){uint16_t t=a;a=b;b=t;}
void MedianFilter3x3(uint16_t* dst,const uint16_t* src,int width,int height){
  for(int i=0;i<height;++i) {
    int im1=i>0?i-1:i;
    int ip1=i<height-1?i+1:i;
    const uint16_t* rm1=&src[im1*width];
    const uint16_t* row=&src[i*width];
    const uint16_t* rp1=&src[ip1*width];

    uint16_t p0,p1,p2,p3,p4,p5,p6,p7,p8;
    {   
      int j=0;
      int jp1=1;
      p0=rm1[j]; p1=rm1[j]; p2=rm1[jp1];
      p3=row[j]; p4=row[j]; p5=row[jp1];
      p6=rp1[j]; p7=rp1[j]; p8=rp1[jp1];

      MFSORT2(p1,p2); MFSORT2(p4,p5); MFSORT2(p7,p8); MFSORT2(p0,p1); MFSORT2(p3,p4); MFSORT2(p6,p7); MFSORT2(p1,p2); MFSORT2(p4,p5);
      MFSORT2(p7,p8); MFSORT2(p0,p3); MFSORT2(p5,p8); MFSORT2(p4,p7); MFSORT2(p3,p6); MFSORT2(p1,p4); MFSORT2(p2,p5); MFSORT2(p4,p7);
      MFSORT2(p4,p2); MFSORT2(p6,p4); MFSORT2(p4,p2);
      dst[i*width+j]=p4;
    }   

    for(int j=1;j<width-1;++j) {
      int jm1=j-1;
      int jp1=j+1;
      p0=rm1[jm1]; p1=rm1[j]; p2=rm1[jp1];
      p3=row[jm1]; p4=row[j]; p5=row[jp1];
      p6=rp1[jm1]; p7=rp1[j]; p8=rp1[jp1];

      MFSORT2(p1,p2); MFSORT2(p4,p5); MFSORT2(p7,p8); MFSORT2(p0,p1); MFSORT2(p3,p4); MFSORT2(p6,p7); MFSORT2(p1,p2); MFSORT2(p4,p5);
      MFSORT2(p7,p8); MFSORT2(p0,p3); MFSORT2(p5,p8); MFSORT2(p4,p7); MFSORT2(p3,p6); MFSORT2(p1,p4); MFSORT2(p2,p5); MFSORT2(p4,p7);
      MFSORT2(p4,p2); MFSORT2(p6,p4); MFSORT2(p4,p2);
      dst[i*width+j]=p4;
    }   

    {   
      int jm1=width-2;
      int j=width-1;
      p0=rm1[jm1]; p1=rm1[j]; p2=rm1[j];
      p3=row[jm1]; p4=row[j]; p5=row[j];
      p6=rp1[jm1]; p7=rp1[j]; p8=rp1[j];

      MFSORT2(p1,p2); MFSORT2(p4,p5); MFSORT2(p7,p8); MFSORT2(p0,p1); MFSORT2(p3,p4); MFSORT2(p6,p7); MFSORT2(p1,p2); MFSORT2(p4,p5);
      MFSORT2(p7,p8); MFSORT2(p0,p3); MFSORT2(p5,p8); MFSORT2(p4,p7); MFSORT2(p3,p6); MFSORT2(p1,p4); MFSORT2(p2,p5); MFSORT2(p4,p7);
      MFSORT2(p4,p2); MFSORT2(p6,p4); MFSORT2(p4,p2);
      dst[i*width+j]=p4;
    }   
  }
}
#undef MFSORT2

