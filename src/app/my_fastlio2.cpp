/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-04 19:46:51
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-04 21:01:37
 */

#include "frontend_wrapper.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "my_fastlio2");
    ros::NodeHandle nh;
    std::shared_ptr<Wrapper::FrontEndWrapper> preprocess_ptr;
    preprocess_ptr = std::make_shared<Wrapper::FrontEndWrapper>(nh);
    return 0;
}