/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-04 11:08:50
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-04 20:02:01
 */
#pragma once
#include <iostream>

namespace IESKFSlam {

    class TimeStamp {
        private:
            uint64_t nsec_;
            double sec_;
        public:
            // 默认单位为 nseconds
            TimeStamp(uint64_t insec = 0) {
                nsec_ = insec;
                sec_ = static_cast<double>(insec) / 1e9;
            }
            void fromSec(double isec) {
                sec_ = isec;
                nsec_ = static_cast<uint64_t>(isec * 1e9);
            }
            void fromNsec(uint64_t insec = 0) {
                nsec_ = insec;
                sec_ = static_cast<double>(insec) / 1e9;
            }
            const uint64_t & nsec() const { return nsec_; }
            const double & sec() const { return sec_; }
    };

} // namespace IESKFSlam