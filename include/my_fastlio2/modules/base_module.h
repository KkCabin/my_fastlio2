/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-04 15:24:14
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-04 19:55:11
 */
#pragma once
#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace IESKFSlam {

    class BaseModule {
        private:
            YAML::Node config_node;
            std::string name;

        protected:
            /**
             * @param {string &} config_path: 参数配置文件地址
             * @param {string &} prefix：     yaml中的前缀
             * @param {string &} module_name  模块的名称
             */
            BaseModule(const std::string & config_path,
                       const std::string & prefix,
                       const std::string & module_name = "default") {
                name = module_name;
                if(config_path != "") {
                    try {
                        config_node = YAML::LoadFile(config_path);
                    } catch (YAML::Exception &e) {
                        std::cout << e.msg << std::endl;
                    }

                    if(prefix != "" && config_node[prefix]) config_node = config_node[prefix];
                }
            }
            /**
             * @param T
             * @param key: 键值
             * @param value: 读取数据到哪个参数
             * @param default_val: 默认值
             */
            template<typename T>
            void readParam(const std::string & key, T & value, T default_value) {
                if(config_node[key]) {
                    value = config_node[key].as<T>();
                } else {
                    value = default_value;
                }
            }
    };

} // namespace IESKFSlam