// PID
  bool *tmp = &need_to_reset_pid_;
  if (t < 0.01) {
    if (need_to_reset_pid_) {
      for (int i = 0; i < 4; ++i) {
        pid_[i].reset();
      }
      *tmp = false;
    }
  } else {
    *tmp = true;
  }

  // float feedback_torque_Nm[4];
  float feedback_tension_N[4];
  for (int i = 0; i < 4; ++i) {
    float lerr = ldes[i] - robot_.len(i);
    feedback_tension_N[i] = pid_[i].update(lerr);
  }

  save_pid_output(feedback_tension_N);

  // WR^-1
  // float feedback_tension_N[4];
  // for (int i = 0; i < 4; ++i) {
  //   feedback_tension_N = feedback_torque_Nm / kR;
  // }
  float W[2][4];
  kinematics_.wrenchMatrix(W);
  float feedback_force_N[2];
  matmul(W, feedback_tension_N, feedback_force_N);
  save_feedback_force(feedback_force_N);

  // feedforward force
  // TODO(gerry): SE(2)
  float ff_force_N[2];
  const float(&vdes_prev)[3] = LQG_GAINS[k == 0 ? k : k - 1].vff;
  static constexpr float kMass = 1.0;
  ff_force_N[0] = (vdes[0] - vdes_prev[0]) / dt * kMass;
  ff_force_N[1] = (vdes[1] - vdes_prev[1]) / dt * kMass;
  float fc_N[2];
  matadd(feedback_force_N, ff_force_N, fc_N);
  save_total_force(fc_N);

  // Tension distribution
  // TODO(gerry): use better tension distribution algorithm
  float tensionTD_Nm[4];
  kinematics_.forceSolverPott(fc_N[0], fc_N[1], W, tensionTD_Nm);
  // SerialD.printf("Tension Distribution: %.3f %.3f %.3f %.3f\n", tensionTD_Nm[0],
  //                tensionTD_Nm[1], tensionTD_Nm[2],
  //                tensionTD_Nm[3]);

  // feedforward torques
  float ldotdes[4];
  float J[4][2];
  kinematics_.jacobian(J);
  matmul(J, vdes, ldotdes);
  float taum_Nm[4];
  for (int i = 0; i < 4; ++i) {
    taum_Nm[i] =
        (tensionTD_Nm[i] - ldotdes[i] * fv_ - tanh(mu_ * ldotdes[i]) * fs_) *
        kR;
  }
  save_torque(taum_Nm);

  // Safety & return
  float torque = taum_Nm[winchnum];
  clamp(&torque, 0.1, 1.2);
  return torque;
}