/// \file: main.cpp
/// \brief: rigid2d_node

#include "ros/ros.h"
#include <iostream>
#include "rigid2d/rigid2d.hpp"

using namespace std;
using namespace rigid2d;

int main(int argc, char **argv){

  	ros::init(argc, argv, "rigid2d_node");
    /// /* 1. Input 2 Transforms.
    ///  */
    cout<<"Please enter two transforms: T_ab and T_bc."<<endl;

    Transform2D T_ab;
    cout<<"Please enter T_ab:"<<endl;
    cin>>T_ab;

    Transform2D T_bc;
    cout<<"Please enter T_bc:"<<endl;
    cin>>T_bc;

    Transform2D T_ac=T_ab*T_bc;//get T_ac
    Transform2D T_ba=T_ab.inv();
    Transform2D T_cb=T_bc.inv();
    Transform2D T_ca=T_ac.inv();

    cout<<"T_ab: "<<T_ab<<endl;
    cout<<"T_ba: "<<T_ba<<endl;
    cout<<"T_bc: "<<T_bc<<endl;
    cout<<"T_cb: "<<T_cb<<endl;
    cout<<"T_ac: "<<T_ac<<endl;
    cout<<"T_ca: "<<T_ca<<endl;

    /// /* 2. Input a Vector/Twist and show the frame change.
    ///  */

    char twist_or_vector;
    bool frame_trans_finish_sign= false;
    char frame_input_sign,frame_output_sign;

    while (frame_trans_finish_sign== false){

        cout<<"Which one do you want to transform? Twist or Vector?"<<endl;
        cout<<"Please answer : 't' or 'v' (t for twist and v for vector).";
        cin>>twist_or_vector;

        if(twist_or_vector=='v'){
            Vector2D vec_for_trans;
            while (frame_trans_finish_sign== false){
                cout<<"Please enter the vector v:"<<endl;
                cin>>vec_for_trans;
                cout<<"input: which frame do you want to input the vector? {a}, {b} or {c}? You could type like: a"<<endl;
                cin>>frame_input_sign;
                cout<<"output: which frame do you want to output the vector? {a}, {b} or {c}? You could type like: b"<<endl;
                cin>>frame_output_sign;

                if(frame_input_sign=='a'){
                    if (frame_output_sign=='a'){
                        cout<<vec_for_trans;
                    }
                    else if(frame_output_sign=='b'){
                        cout<<T_ba(vec_for_trans);
                    }
                    else if(frame_output_sign=='c'){
                        cout<<T_ca(vec_for_trans);
                    }
                    else{
                        cout<<"Wrong output message. Please type a, b or c."<<endl;
                        continue;
                    }
                }
                else if (frame_input_sign=='b'){
                    if (frame_output_sign=='a'){
                        cout<<T_ab(vec_for_trans);
                    }
                    else if(frame_output_sign=='b'){
                        cout<<vec_for_trans;
                    }
                    else if(frame_output_sign=='c'){
                        cout<<T_cb(vec_for_trans);
                    }
                    else{
                        cout<<"Wrong output message. Please type a, b or c."<<endl;
                        continue;
                    }
                }
                else if (frame_input_sign=='c'){
                    if (frame_output_sign=='a'){
                        cout<<T_ac(vec_for_trans);
                    }
                    else if(frame_output_sign=='b'){
                        cout<<T_bc(vec_for_trans);
                    }
                    else if(frame_output_sign=='c'){
                        cout<<vec_for_trans;
                    }
                    else{
                        cout<<"Wrong output message. Please type a, b or c."<<endl;
                        continue;
                    }
                }
                else{
                    cout<<"Wrong input message. Please type a, b or c."<<endl;
                    continue;
                }
                frame_trans_finish_sign= true;

            }
        }
        else if(twist_or_vector=='t') {
            Twist2D twist_for_trans;

            while (frame_trans_finish_sign== false){
                cout<<"Please enter the twist t:"<<endl;
                cin>>twist_for_trans;
                cout<<"input: which frame do you want to input the twist? {a}, {b} or {c}? You could type like: a"<<endl;
                cin>>frame_input_sign;
                cout<<"output: which frame do you want to output the twist? {a}, {b} or {c}? You could type like: b"<<endl;
                cin>>frame_output_sign;

                if(frame_input_sign=='a'){
                    if (frame_output_sign=='a'){
                        cout<<twist_for_trans;
                    }
                    else if(frame_output_sign=='b'){
                        cout<<T_ba(twist_for_trans);
                    }
                    else if(frame_output_sign=='c'){
                        cout<<T_ca(twist_for_trans);
                    }
                    else{
                        cout<<"Wrong output message. Please type a, b or c."<<endl;
                        continue;
                    }
                }
                else if (frame_input_sign=='b'){
                    if (frame_output_sign=='a'){
                        cout<<T_ab(twist_for_trans);
                    }
                    else if(frame_output_sign=='b'){
                        cout<<twist_for_trans;
                    }
                    else if(frame_output_sign=='c'){
                        cout<<T_cb(twist_for_trans);
                    }
                    else{
                        cout<<"Wrong output message. Please type a, b or c."<<endl;
                        continue;
                    }
                }
                else if (frame_input_sign=='c'){
                    if (frame_output_sign=='a'){
                        cout<<T_ac(twist_for_trans);
                    }
                    else if(frame_output_sign=='b'){
                        cout<<T_bc(twist_for_trans);
                    }
                    else if(frame_output_sign=='c'){
                        cout<<twist_for_trans;
                    }
                    else{
                        cout<<"Wrong output message. Please type a, b or c."<<endl;
                        continue;
                    }
                }
                else{
                    cout<<"Wrong input message. Please type a, b or c."<<endl;
                    continue;
                }
                frame_trans_finish_sign= true;

            }
        }
        else{
            cout<<"Wrong input message. Please type 't' or 'v'(t for twist and v for vector). Example: t"<<endl;
            continue;
        }
    }


    return 0;
}
