#pragma once 

#include <Eigen/Dense>

#include "imu_gps_localizer/base_type.h"

namespace ImuGpsLocalization {
//hayden： GpsProcessor类通过GPS位置更新系统的状态UpdateStateByGpsPosition()以及计算残差以及雅可比矩阵和把得到的误差更新状态
class GpsProcessor {
public:
    GpsProcessor(const Eigen::Vector3d& I_p_Gps);

    bool UpdateStateByGpsPosition(const Eigen::Vector3d& init_lla, const GpsPositionDataPtr gps_data_ptr, State* state);

private:    
    void ComputeJacobianAndResidual(const Eigen::Vector3d& init_lla,  
                                    const GpsPositionDataPtr gps_data, 
                                    const State& state,
                                    Eigen::Matrix<double, 3, 15>* jacobian,
                                    Eigen::Vector3d* residual);

    const Eigen::Vector3d I_p_Gps_;  
};

void AddDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x, State* state);

}  // namespace ImuGpsLocalization