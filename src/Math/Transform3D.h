#ifndef TRANSFORM_3D_H_SEPT_9_2007_SVL5
#define TRANSFORM_3D_H_SEPT_9_2007_SVL5

#include "Common.h"
#include "Mat4x4d.h"
#include "Mat3x3D.h"
#include "V3D.h"
#include <assert.h>

//#include <boost/numeric/ublas/matrix.hpp>
#include <string>
using namespace std;

class Attitude;

class Transform3D{
public:
  Transform3D(){
    eye4x4(xform);
  }

  Transform3D(const double mat[16]){
    memcpy(xform,mat,sizeof(double)*16);
  }

  void Reset(){
    eye4x4(xform);
  }

  // right-handed rotation about an arbitrary axis
  void Rotate(V3D axis, double thetaRads)
  {
    axis = axis.norm();
    double R[16];
    eye4x4(R);
  
    // c = cos (theta), s = sin (theta), t = 1-cos (theta), and <X,Y,Z> is the unit vector representing the arbitary axis
    double c = cos(thetaRads);
    double s = sin(thetaRads);
    double t = (1.0-cos(thetaRads));
    double X=axis.x, Y=axis.y, Z=axis.z;

    R[0]  = t*X*X + c;
    R[1]  = t*X*Y + s*Z;
    R[2]  = t*X*Z - s*Y;

    R[4]  = t*X*Y - s*Z;
    R[5]  = t*Y*Y + c;
    R[6]  = t*Y*Z + s*X;

    R[8]  = t*X*Y + s*Y;
    R[9]  = t*Y*Z - s*X;
    R[10] = t*Z*Z + c;

    matmul4x4(xform,R,xform);
  }

  void Translate(double x, double y, double z){
    xform[3] += x;
    xform[7] += y;
    xform[11] += z;
  }

  Transform3D Transpose(){
    Transform3D ret;
    for(int x=0;x<4;x++){
      for(int y=0;y<4;y++){
        ret.xform[y*4+x] = xform[x*4+y];
      }
    }
    return ret;
  }

  // rotation applied in this order: R, then P, then Y
  void Rotate(double yaw, double pitch, double roll){
    double R[16];
   
    // Roll
    if(roll!=0){
      eye4x4(R);
      R[5] = cos(roll);
      R[6] = sin(roll);
      R[9] = -sin(roll);
      R[10] = cos(roll);
      matmul4x4(xform,R,xform);
    }

    // Pitch
    if(pitch!=0){
      eye4x4(R);
      R[0] = cos(pitch);
      R[2] = -sin(pitch);
      R[8] = sin(pitch);
      R[10] = cos(pitch);
      matmul4x4(xform,R,xform);
    }

    // Yaw
    if(yaw!=0){
      eye4x4(R);
      R[0] = cos(yaw);
      R[1] = sin(yaw);
      R[4] = -sin(yaw);
      R[5] = cos(yaw);
      matmul4x4(xform,R,xform);
    }
  }

    // rotation applied in this order: Y, then P, then R
  void RotateYPR(double yaw, double pitch, double roll){
    double R[16];
   
    // Yaw
    if(yaw!=0){
      eye4x4(R);
      R[0] = cos(yaw);
      R[1] = sin(yaw);
      R[4] = -sin(yaw);
      R[5] = cos(yaw);
      matmul4x4(xform,R,xform);
    }

    // Pitch
    if(pitch!=0){
      eye4x4(R);
      R[0] = cos(pitch);
      R[2] = -sin(pitch);
      R[8] = sin(pitch);
      R[10] = cos(pitch);
      matmul4x4(xform,R,xform);
    }
    
    // Roll
    if(roll!=0){
      eye4x4(R);
      R[5] = cos(roll);
      R[6] = sin(roll);
      R[9] = -sin(roll);
      R[10] = cos(roll);
      matmul4x4(xform,R,xform);
    }
  }

  Transform3D Inverse() const{
    Transform3D ret(*this);
    ret.Invert();
    return ret;
  }

  void Invert(){
    double tmp[16];
    unsigned char r,c;

    memcpy(tmp,xform,sizeof(double)*16);

    // figure out the inverse
    for (r = 0; r < 3; r++){
      for (c = 0; c < 3; c++){
        //upper left 3x3 block is just transposed
        xform[c*4+r] = tmp[r*4+c];
      }
    }
    //last row is [0, 0, 0, 1]
    xform[12] = xform[13] = xform[14] = 0.0;
    xform[15] = 1.0;

    //last col is -R'*d
    for (r = 0; r < 3; r++)
    {
      xform[r*4+3] = 0.0; // init last column to 0
      for (c = 0; c < 3; c++)
        xform[r*4+3] -= tmp[c*4+r] * tmp[c*4+3];
    }
  }

  const double* GetXFormMatrix_RowMajor() const{
    return xform;
  }

  void GetXFormMatrix_RowMajor(double dest[16]) const{
    memcpy(dest,xform,sizeof(double)*16);
  }

  void GetXFormMatrix_ColMajor(double dest[16]) const{
    for(unsigned char r=0;r<4;r++){
      for(unsigned char c=0;c<4;c++){
        dest[c*4+r] = xform[r*4+c];
      }
    }
  }

  // combines this transform with "b"
  Transform3D operator*(const Transform3D& b){
    Transform3D ret(*this);
    matmul4x4(xform,b.xform,ret.xform);
    return ret;
  }

  // combines this transform with "b"
  Transform3D operator*(const double b[16]){
    Transform3D ret(*this);
    matmul4x4(xform,b,ret.xform);
    return ret;
  }

  // v==1 --> use this transform; v==0 --> use 'b' transform
  Transform3D Interpolate(const Transform3D b, double v){
    Transform3D ret(*this);
    for(unsigned char i=0;i<16;i++){
      ret.xform[i] = ret.xform[i]*v + b.xform[i]*(1.0-v);
    }
    return ret;
  }

  V3D Transform(V3D in)
  {
    double tmp[4];    

    tmp[0] = in.x*xform[0]+in.y*xform[1]+in.z*xform[2]+xform[3];
    tmp[1] = in.x*xform[4]+in.y*xform[5]+in.z*xform[6]+xform[7];
    tmp[2] = in.x*xform[8]+in.y*xform[9]+in.z*xform[10]+xform[11];
    tmp[3] = in.x*xform[12]+in.y*xform[13]+in.z*xform[14]+xform[15];

    return V3D(tmp[0]/tmp[3],tmp[1]/tmp[3],tmp[2]/tmp[3]);
  }

  // row-major!
  double& at(int row, int col)
  {
    assert(0<=row && row<4);
    assert(0<=col && col<4);
    return xform[row*4+col];
  }
  
  
  double xform[16];
  bool valid;
};

V3D RotateEulerYPR(V3D in, double yaw, double pitch, double roll);
V3D RotateEulerYPR(V3D in, const Attitude& att);
V3D InvRotateEulerYPR(V3D in, const Attitude& att);
V3D RotateEulerYPR2(V3D in, double yaw, double pitch, double roll);
V3D RotateEulerYPR2Inv(V3D in, V3D ypr);



// Code for quaterinions and general rotations (Mike Sherback legacy)
/*BNUV<double,3>   CrossProdCorrect(const BNUV<double,3>& a, const BNUV<double,3>& b); // cross product for two vectors
BNUV<double,4>   QuatMult(const BNUV<double,4>& a,const BNUV<double,4>& b); // quaternion multiplication
BNUV<double,4>   QuatMult2(const BNUV<double,4>& a,const BNUV<double,4>& b); // quaternion multiplication (Note: different notation than Diebel)
BNUV<double,4>   QuatInv(const BNUV<double,4> &a); // inverse quaternion
BNUM<double,3,3> Quat_To_Rot(const BNUV<double,4>& q);
BNUM<double,3,3> Quat_To_Rot2(const BNUV<double,4>& q); //(Note: different notation than Diebel)
void             Quat_To_Euler(BNUV<double,4> q,  double &yaw, double &pitch, double &roll);
BNUV<double,4>   Rot_To_Quat (const BNUM<double,3,3>& R);
BNUV<double,4>   Rot_To_Nearest_Quat(const BNUM<double,3,3>& T, const BNUV<double,4>& oldQ); // deals with ambiguity by picking the quaternion closest to the previous
void             Rot_To_Euler(BNUM<double,3,3> R, double &yaw, double &pitch, double &roll);
BNUM<double,3,3> Euler_To_Rot (double yaw,  double pitch,  double roll);
BNUV<double,4>   Euler_To_Quat(double y2,  double p2,  double r2);
BNUV<double,4>   Euler_To_Quat2(double y2,  double p2,  double r2); //(Note: different notation than Diebel)
BNUV<double,4>   Euler_To_Nearest_Quat(double y2,  double p2,  double r2, const BNUV<double,4>& oldQ); // deals with ambiguity by picking the quaternion closest to the previous

//-----------------------------------------------------
// code for quaternion rates and angular velocity
BNUV<double,3>  QuatQDot_To_Omega(BNUV<double,4> q, BNUV<double,4> qd);
BNUV<double,3>  QuatQuatDt_To_Omega(BNUV<double,4> oldQ, BNUV<double,4> newQ, double dt);*/

/**
 * Updated attitude (in quaternion form) given a set of body rates and a time 
 * duration.
 *
 * Takes an existing attitude, in normalized quaternion form and returns a new
 * attitude by applying the given body rates for a given duration of time.
 *
 * @param q
 *   Current attitude, represented as a normalized quaternion
 * @param wBody
 *   Body rates (in a body coordinate frame), rad/s
 * @param $dt
 *   Duration of time, in seconds
 *
 * @return
 *   Returns a new attitude in normalized quaternion form
 *
 * @author This written by sergei. Function originally written by Mike Sherback.
 */
//BNUV<double,4>  QuatOmegaDt_To_Quat(const BNUV<double,4>& q, const BNUV<double,3>& wBody, double dt);

// interpolate between two quaternions along the torque-minimal path between q0 and q1
// see http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/
// this is probably not the most efficient way to implement it
// q0 and q1 should be unit quaternions.
// t should be between 0 and 1 (it's constrained internally)
// t = 0 returns q0; t=1 returns q1; something inbetween returns something inbetween
//BNUV<double,4> Quat_Interpolate_SLERP(const BNUV<double,4>& q0, const BNUV<double,4>& q1, double t);

#endif //TRANSFORM_3D_H_SEPT_9_2007_SVL5
