#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>

using namespace std;

class Trajectory {
    public:
        struct State {
            double q;
            double dq;
            double ddq;
            double dddq;
        };
        


        State cubic_trajectory_generator(State state0, State state1, double max_acceleration, double max_velocity, double dt) {
            double current_position = state0.q;
            double current_velocity = state0.dq;
            double target_position = state1.q;
            double target_velocity = state1.dq;
            State state;
            state.q = 0.0;
            state.dq = 0.0;
            // 计算当前位置与目标位置的距离
            double distance = target_position - current_position;
            std::cout << std::endl << "Distance is: " << distance << std::endl;
            // 计算当前速度与目标速度之间的差值
            double velocity_diff = target_velocity - current_velocity;
            std::cout << std::endl << "Velocity diff: " << velocity_diff << std::endl;
            if (abs(velocity_diff) >= 0.000005) {
                
                // 计算加速度方向
                double acceleration_sign = (velocity_diff >= 0) ? 1 : -1;
                
                // 计算在当前位置下可以加速或减速到目标速度的最大加速度
                double max_possible_acceleration = min(max_acceleration, abs(velocity_diff) / dt);
                std::cout << std::endl << "Next max_possible_acceleration is: " << max_possible_acceleration << std::endl;
                
                // 根据距离计算加速度大小
                double acceleration = min(max_possible_acceleration, sqrt(2 * max_acceleration * abs(distance)));
                
                // 根据加速度方向调整加速度
                acceleration = acceleration * acceleration_sign;
                std::cout << std::endl << "Next acc is: " << acceleration << std::endl;
                // 根据当前速度和加速度计算下一个时间点的速度
                double next_velocity = current_velocity + acceleration * dt;
                std::cout << std::endl << "Next velocity is: " << next_velocity << std::endl;
                // 限制速度在最大速度范围内
                next_velocity = min(max(next_velocity, -max_velocity), max_velocity);
                std::cout << std::endl << "Next velocity is: " << next_velocity << std::endl;

                // calculate next position
                // double next_position = current_position + (current_velocity + next_velocity)/2 * dt;
                double next_position = current_position + acceleration/2 * dt*dt;
                state.q = next_position ;
                state.dq = next_velocity;
                return state;
            }
            else {
                // 计算加速度方向
                double acceleration_sign = -(distance >= 0) ? 1 : -1;
                
                // 计算在当前位置下可以加速或减速到目标速度的最大加速度
                double max_possible_acceleration = max_acceleration;
                std::cout << std::endl << "Next max_possible_acceleration is: " << max_possible_acceleration << std::endl;
                
                // 根据距离计算加速度大小
                double acceleration = min(max_possible_acceleration, sqrt(2 * max_acceleration * abs(distance)));
                
                // 根据加速度方向调整加速度
                acceleration = acceleration * acceleration_sign;
                std::cout << std::endl << "Next acc is: " << acceleration << std::endl;
                // 根据当前速度和加速度计算下一个时间点的速度
                double next_velocity = current_velocity + acceleration * dt;
                std::cout << std::endl << "Next velocity is: " << next_velocity << std::endl;
                // 限制速度在最大速度范围内
                next_velocity = min(max(next_velocity, -max_velocity), max_velocity);
                std::cout << std::endl << "Next velocity is: " << next_velocity << std::endl;

                // calculate next position
                double next_position = current_position + acceleration/2 * dt*dt;

                state.q = next_position ;
                state.dq = next_velocity;
                return state;
            }
            

            
        }

        State OnlinePlanning(State state0, State state1, double T, double Ts) {
            //T is the total time, Ts is the sampling time
            double q0 = state0.q;
            double q1 = state1.q;
            int delta = (q1 - q0) > 0 ? 1 : -1; // delta = 1 or -1

            q0 = delta * state0.q;
            q1 = delta * state1.q;
            double v0 = delta * state0.dq;
            double v1 = delta * state1.dq;
            double a0 = delta * state0.ddq;
            double a1 = delta * state1.ddq;
            double j0 = delta * state0.dddq;
            double j1 = delta * state1.dddq;


            double v_max_ = 0.3;
            double a_max_ = 2;
            double j_max_ = 1;
            double v_min_ = -0.3;
            double a_min_ = -2;
            double j_min_ = -1;

            double v_max = (delta + 1) / 2 * v_max_ + (delta - 1) / 2 * v_min_;
            double a_max = (delta + 1) / 2 * a_max_ + (delta - 1) / 2 * a_min_;
            double j_max = (delta + 1) / 2 * j_max_ + (delta - 1) / 2 * j_min_;
            double v_min = (delta + 1) / 2 * v_min_ + (delta - 1) / 2 * v_max_;
            double a_min = (delta + 1) / 2 * a_min_ + (delta - 1) / 2 * a_max_;
            double j_min = (delta + 1) / 2 * j_min_ + (delta - 1) / 2 * j_max_;

            int k = 0;
            int size0 = T / Ts;
            State state;
            state.q = 0.0;
            state.dq = 0.0;
            state.ddq = 0.0;
            state.dddq = 0.0;

            state.q = q0;
            state.dq = v0;
            state.ddq = a0;
            state.dddq = j0;

            double EPSILON = 0.0;

            for (int i = 2; i < size0; ++i) {
                double T_j2a = (a_min - state.ddq) / j_min;
                double T_j2b = (a1 - a_min) / j_max;
                double T_d = (v1 - state.dq) / a_min + T_j2a * (a_min - state.ddq) / (2 * a_min) + T_j2b * (a_min - a1) / (2 * a_min);
                
                if (T_d < T_j2a + T_j2b) {
                    T_j2a = -state.ddq / j_min + sqrt((j_max - j_min) * (state.ddq * state.ddq * j_max - j_min * (a1 * a1 + 2 * j_max * (state.dq - v1)))) / (j_min * (j_min - j_max));
                    T_j2b = a1 / j_max + sqrt((j_max - j_min) * (state.ddq * state.ddq * j_max - j_min * (a1 * a1 + 2 * j_max * (state.dq - v1)))) / (j_max * (j_max - j_min));
                    T_d = T_j2a + T_j2b;
                }

                double h = 0.5 * state.ddq * T_d * T_d + (j_min * T_j2a * (3 * T_d * T_d - 3 * T_d * T_j2a + T_j2a * T_j2a) + j_max * T_j2b * T_j2b * T_j2b) / 6 + T_d * state.dq;
                
                if (h <= (q1 - state.q) - EPSILON) {
                    if ((state.dq - state.ddq * state.ddq / (2 * j_min) < v_max - EPSILON) && (state.ddq < a_max - EPSILON))
                        state.dddq = j_max;
                    else if ((state.dq - state.ddq * state.ddq / (2 * j_min) >= v_max + EPSILON) && (state.ddq > 0 + EPSILON))
                        state.dddq = j_min;
                    else
                        state.dddq = 0;
                    k = 0;

                } else {
                    k = i;
                    if ((i - k) >= 0 && (i - k) <= T_j2a / Ts)
                        state.dddq = j_min;
                    else if ((i - k) <= T_d / Ts && (i - k) >= (T_d - T_j2b) / Ts)
                        state.dddq = j_max;
                    else
                        state.dddq = 0;
                }

                if (fabs(state.q - q1) < 1e-5 && fabs(state.dq - v1) < 1e-3 && fabs(state.ddq - a1) < 1e-2) {
                    state.dq = 0;
                    state.ddq = 0;
                    state.dddq = 0;
                    state.dddq = 0;
                }
                state.ddq = state.ddq + Ts / 2 * (state.dddq + state.dddq);
                state.dq = state.dq + Ts / 2 * (state.ddq + state.ddq);
                state.q = state.q + Ts / 2 * (state.dq + state.dq);
            }

            state.q = delta * state.q;
            state.dq = delta * state.dq;
            state.ddq = delta * state.ddq;
            state.dddq = delta * state.dddq;

            return state;
        }
};