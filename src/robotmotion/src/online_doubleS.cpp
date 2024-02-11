#include <vector>
#include <cmath>
class Trajectory {
    public:
        struct State {
            double q;
            double dq;
            double ddq;
            double dddq;
        };

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
            double j_max_ = 10;
            double v_min_ = -0.3;
            double a_min_ = -2;
            double j_min_ = -10;

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