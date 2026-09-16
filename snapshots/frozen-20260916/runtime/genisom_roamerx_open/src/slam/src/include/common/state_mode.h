/**
 * @file state_mode.h
 * @brief
 * @author Liuzhao Li (liliuzhao@jushenzhiren.com)
 * @version 1.0
 * @date 2025-08-04
 * @copyright Copyright (C) 2025 具身智人(北京)科技有限公司
 */

#pragma once

namespace robot::slam
{
    enum class SlamState
    {
        STABLE,
        PASSIVE,
        READY,    // The SLAM system is ready to start processing.
        ACTIVE,   // The SLAM system is actively running and processing data.
        ERROR,    // The SLAM system encountered an error.
        SAVE,     // The SLAM system encountered an save.
        SUCCESS,  // The SLAM system has successfully completed its operations.
        OTHERS    // The SLAM system encountered an others.
    };
}  // namespace robot::slam