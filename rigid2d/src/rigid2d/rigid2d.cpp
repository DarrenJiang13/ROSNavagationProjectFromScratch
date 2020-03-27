/// \file rigid2d.cpp
/// \brief implementation of rigid2d.hpp


#include "rigid2d/rigid2d.hpp" // include the header file
#include <cmath>
#include <vector>
#include <iostream>
using namespace rigid2d;
using namespace std;

///// refer: https://github.com/SorinaStoian/Tema1-EGC/blob/9b3278a9d0c8156a7b7445bcfb8f1a6743b2ca3e/lab2/Framework/Transform2D.cpp

///// static_assertions test compile time assumptions.
///// You should write at least one more test for each function
static_assert(almost_equal(0, 0), "is_zero failed");
static_assert(almost_equal(0.001, 0.005, 1.0e-2), "is_zero failed");
static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");
static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg failed");

Transform2D::Transform2D(){
    TransformMatrix[0][0] = 1;	TransformMatrix[0][1] = 0;	TransformMatrix[0][2] = 0;
    TransformMatrix[1][0] = 0;	TransformMatrix[1][1] = 1;	TransformMatrix[1][2] = 0;
    TransformMatrix[2][0] = 0;	TransformMatrix[2][1] = 0;	TransformMatrix[2][2] = 1;
}

Transform2D::Transform2D(const Vector2D & trans){

    TransformMatrix[0][0] = 1;	TransformMatrix[0][1] = 0;	TransformMatrix[0][2] = trans.x;
    TransformMatrix[1][0] = 0;	TransformMatrix[1][1] = 1;	TransformMatrix[1][2] = trans.y;
    TransformMatrix[2][0] = 0;	TransformMatrix[2][1] = 0;	TransformMatrix[2][2] = 1;
}

Transform2D::Transform2D(double radians){

    TransformMatrix[0][0] = cos(radians);	TransformMatrix[0][1] = - sin(radians);	TransformMatrix[0][2] = 0;
    TransformMatrix[1][0] = sin(radians);	TransformMatrix[1][1] = cos(radians);	TransformMatrix[1][2] = 0;
    TransformMatrix[2][0] = 0;	TransformMatrix[2][1] = 0;	TransformMatrix[2][2] = 1;
}

Transform2D::Transform2D(const Vector2D & trans, double radians){

    TransformMatrix[0][0] = cos(radians);	TransformMatrix[0][1] = - sin(radians);	TransformMatrix[0][2] = trans.x;
    TransformMatrix[1][0] = sin(radians);	TransformMatrix[1][1] = cos(radians);	TransformMatrix[1][2] = trans.y;
    TransformMatrix[2][0] = 0;	TransformMatrix[2][1] = 0;	TransformMatrix[2][2] = 1;
}

Transform2D::Transform2D(Twist2D pose){
    TransformMatrix[0][0] = cos(pose.w);	TransformMatrix[0][1] = - sin(pose.w);	TransformMatrix[0][2] = pose.vx;
    TransformMatrix[1][0] = sin(pose.w);	TransformMatrix[1][1] = cos(pose.w);	TransformMatrix[1][2] = pose.vy;
    TransformMatrix[2][0] = 0;	TransformMatrix[2][1] = 0;	TransformMatrix[2][2] = 1;
}

Vector2D Transform2D::operator()(Vector2D v) const{

    Vector2D returnVec;
    returnVec.x= TransformMatrix[0][0]*v.x+TransformMatrix[0][1]*v.y+TransformMatrix[0][2];
    returnVec.y= TransformMatrix[1][0]*v.x+TransformMatrix[1][1]*v.y+TransformMatrix[1][2];
    return returnVec;
}

Twist2D Transform2D::operator()(Twist2D t) const{

    Twist2D returnTwist;
    returnTwist.w=t.w;
    returnTwist.vx=t.w*TransformMatrix[1][2]+TransformMatrix[0][0]*t.vx+TransformMatrix[0][1]*t.vy;
    returnTwist.vy=-t.w*TransformMatrix[0][2]+TransformMatrix[1][0]*t.vx+TransformMatrix[1][1]*t.vy;

    return returnTwist;
}

Transform2D Transform2D::inv() const{
    Transform2D InvTrans2D;

    InvTrans2D.TransformMatrix[0][0] = TransformMatrix[0][0];
    InvTrans2D.TransformMatrix[0][1] = TransformMatrix[1][0];
    InvTrans2D.TransformMatrix[0][2] = -(TransformMatrix[0][0]*TransformMatrix[0][2]+TransformMatrix[1][0]*TransformMatrix[1][2]);
    InvTrans2D.TransformMatrix[1][0] = TransformMatrix[0][1];
    InvTrans2D.TransformMatrix[1][1] = TransformMatrix[1][1];
    InvTrans2D.TransformMatrix[1][2] = -(TransformMatrix[0][1]*TransformMatrix[0][2]+TransformMatrix[1][1]*TransformMatrix[1][2]);
    InvTrans2D.TransformMatrix[2][0] = 0;
    InvTrans2D.TransformMatrix[2][1] = 0;
    InvTrans2D.TransformMatrix[2][2] = 1;

    return InvTrans2D;
}

Twist2D Transform2D::displacement() {
    Twist2D returnTwist;
    double cos_theta=TransformMatrix[0][0];
    double sin_theta=TransformMatrix[1][0];
    double x=TransformMatrix[0][2];
    double y=TransformMatrix[1][2];
    double theta_radians;

    if (sin_theta>0){
        theta_radians=acos(cos_theta);
    }
    else{
        theta_radians=2*PI-acos(cos_theta);
    }

    if ((cos_theta>0) & (abs(sin_theta)<0.0001)){
        theta_radians=0;
    }

    returnTwist.w=theta_radians;
    returnTwist.vx=x;
    returnTwist.vy=y;

    return returnTwist;
}

Transform2D & Transform2D::operator*=(const Transform2D & rhs){

    //Transform2D transform_tmp;

    double a11,a12,a13,a21,a22,a23,a31,a32,a33;
    double b11,b12,b13,b21,b22,b23,b31,b32,b33;
    a11=TransformMatrix[0][0];  a12=TransformMatrix[0][1];  a13=TransformMatrix[0][2];
    a21=TransformMatrix[1][0];  a22=TransformMatrix[1][1];  a23=TransformMatrix[1][2];
    a31=TransformMatrix[2][0];  a32=TransformMatrix[2][1];  a33=TransformMatrix[2][2];

    b11=rhs.TransformMatrix[0][0];  b12=rhs.TransformMatrix[0][1];  b13=rhs.TransformMatrix[0][2];
    b21=rhs.TransformMatrix[1][0];  b22=rhs.TransformMatrix[1][1];  b23=rhs.TransformMatrix[1][2];
    b31=rhs.TransformMatrix[2][0];  b32=rhs.TransformMatrix[2][1];  b33=rhs.TransformMatrix[2][2];


    TransformMatrix[0][0]=a11*b11+a12*b21+a13*b31;
    TransformMatrix[0][1]=a11*b12+a12*b22+a13*b32;
    TransformMatrix[0][2]=a11*b13+a12*b23+a13*b33;
    TransformMatrix[1][0]=a21*b11+a22*b21+a23*b31;
    TransformMatrix[1][1]=a21*b12+a22*b22+a23*b32;
    TransformMatrix[1][2]=a21*b13+a22*b23+a23*b33;
    TransformMatrix[2][0]=a31*b11+a32*b21+a33*b31;
    TransformMatrix[2][1]=a31*b12+a32*b22+a33*b32;
    TransformMatrix[2][2]=a31*b13+a32*b23+a33*b33;

    return *this;
}

std::istream & rigid2d::operator>>(std::istream & is, Vector2D & v){
    cout<<"input the vector.x: ";
    is>>v.x;
    cout<<"input the vector.y: ";
    is>>v.y;
    return is;
}

std::ostream & rigid2d::operator<<(std::ostream & os, const Vector2D & v){
    os<<"[ "<<v.x<<" "<<v.y<<" ]"<<endl;
    return os;
}

std::istream & rigid2d::operator>>(std::istream & is, Twist2D & t){
    cout<<"input the twist.w: ";
    is>>t.w;
    cout<<"input the twist.vx: ";
    is>>t.vx;
    cout<<"input the twist.vy: ";
    is>>t.vy;
    return is;
}

std::ostream & rigid2d::operator<<(std::ostream & os, const Twist2D & t){
    os<<"[ "<<t.w<<" "<<t.vx<<" "<<t.vy<<" ]"<<endl;
    return os;
}

std::istream & rigid2d::operator>>(std::istream & is, Transform2D & tf){
    Vector2D v_tmp;
    double degree,radians;

    cout<<"input the degrees: ";
    is>>degree;
    cout<<"input the vector.x: ";
    is>>v_tmp.x;
    cout<<"input the vector.y: ";
    is>>v_tmp.y;

    radians=deg2rad(degree);
    Transform2D tf_input(v_tmp,radians);

    tf*=tf_input;
    cout<<"Matrix created.\n";

    return is;
}

std::ostream & rigid2d::operator<<(std::ostream & os, const Transform2D & tf){
    double cos_theta=tf.TransformMatrix[0][0];
    double sin_theta=tf.TransformMatrix[1][0];
    double x=tf.TransformMatrix[0][2];
    double y=tf.TransformMatrix[1][2];
    double theta_radians;

    if (sin_theta>0){
        theta_radians=acos(cos_theta);
    }
    else{
        theta_radians=2*PI-acos(cos_theta);
    }

    if ((cos_theta>0) & (abs(sin_theta)<0.0001)){
        theta_radians=0;
    }
//    Twist2D twist=tf.displacement();
//    os<<"theta(in radians): "<<twist.w<<" x: "<<twist.vx<<" y: "<<twist.vy<<endl;

    os<<"theta(in radians): "<<theta_radians<<" x: "<<x<<" y: "<<y<<endl;
    return os;
}

Transform2D rigid2d::operator*(Transform2D lhs, const Transform2D & rhs){

    Transform2D transform_tmp=lhs;
    transform_tmp*=rhs;

    return transform_tmp;
}

Vector2D rigid2d::operator+=(Vector2D & lhs, const Vector2D & rhs){
    lhs.x=lhs.x+rhs.x;
    lhs.y=lhs.y+rhs.y;
    return lhs;
}

Vector2D rigid2d::operator+(Vector2D lhs, const Vector2D & rhs){
    Vector2D returnVec(lhs.x+rhs.x,lhs.y+rhs.y);
    return returnVec;
}

Vector2D rigid2d::operator-=(Vector2D & lhs, const Vector2D & rhs){
    lhs.x=lhs.x-rhs.x;
    lhs.y=lhs.y-rhs.y;
    return lhs;
}

Vector2D rigid2d::operator-(Vector2D lhs, const Vector2D & rhs){
    Vector2D returnVec(lhs.x-rhs.x,lhs.y-rhs.y);
    return returnVec;
}

Vector2D rigid2d::operator*=(Vector2D & lhs, const double rhs){
    lhs.x=lhs.x*rhs;
    lhs.y=lhs.y*rhs;
    return lhs;
}

Vector2D rigid2d::operator*(Vector2D & lhs, const double rhs){
    Vector2D returnVec(lhs.x*rhs,lhs.y*rhs);
    return returnVec;
}

Vector2D rigid2d::operator*=(const double lhs, Vector2D & rhs){
    rhs.x=rhs.x*lhs;
    rhs.y=rhs.y*lhs;
    return rhs;
}

Vector2D rigid2d::operator*(const double lhs, Vector2D & rhs){
    Vector2D returnVec(rhs.x*lhs,rhs.y*lhs);
    return returnVec;
}

Vector2D rigid2d::normalize(Vector2D vec){

    Vector2D returnVec;

    double vec_norm=sqrt(vec.x*vec.x+vec.y*vec.y);
    returnVec.x=vec.x/vec_norm;
    returnVec.y=vec.y/vec_norm;

    return returnVec;
}

Transform2D rigid2d::integrateTwist(Twist2D twist){

    Vector2D vec;
    double radians;

    //!!!
    radians=twist.w*1;
    if (abs(radians) <0.001){
        vec.x=twist.vx;
        vec.y=-twist.vy;
    }
    else{
        vec.x=twist.vx*sin(radians)+twist.vy*cos(radians)-twist.vy;
        vec.y=-twist.vx*cos(radians)+twist.vy*sin(radians)+twist.vx;
    }

    Transform2D tf(vec,radians);
    return tf;
}

double rigid2d::length(Vector2D vec){
    double len;
    len=sqrt(vec.x*vec.x+vec.y*vec.y);
    return len;
}

double rigid2d::distance(Vector2D vec1, Vector2D vec2){
    double dis;
    dis=sqrt((vec1.x-vec2.x)*(vec1.x-vec2.x)+(vec1.y-vec2.y)*(vec1.y-vec2.y));
    return dis;
}

double rigid2d::angle(Vector2D vec){

    // in radians

    double radians=atan2(vec.y,vec.x);

    return radians;
}

